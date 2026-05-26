/**
 * @file main.c
 * @brief ESP32-S3 master firmware for the Breville Profusion espresso machine.
 *
 * Two-MCU design: this ESP32-S3 handles the display (360×360 LVGL UI), brew
 * scheduling, SD logging, WiFi/NTP, and power management.  An Arduino Uno slave
 * at I2C address 0x08 owns the sensors (pressure, temperatures) and PID control
 * of the pump and thermoblock.
 *
 * All I2C operations — sensor reads, debug telemetry, and PID commands — are
 * serialised through a single FreeRTOS queue (i2c_queue) and executed by one
 * dedicated task (i2c_communication_task).  This prevents bus contention between
 * the LVGL render path and any caller that needs to write to the Arduino.
 */

#include "ST77916.h"
#include "PCF85063.h"
#include "SD_MMC.h"
#include "Wireless.h"
#include "TCA9554PWR.h"
#include "LVGL_UI/LVGL_Coffee_UI.h"

#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>
#include <sys/stat.h> 
#include <unistd.h> 

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_pm.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "profile_manager.h" 
#include "time_sync.h"

// === MACRO SAFEGUARDS ===
#ifdef I2C_MASTER_NUM
#undef I2C_MASTER_NUM
#endif
#ifdef I2C_MASTER_FREQ_HZ
#undef I2C_MASTER_FREQ_HZ
#endif

// Arduino I2C Configuration
#define ARDUINO_I2C_ADDRESS     0x08
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      50000      
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS   1000

#define I2C_MASTER_SDA_IO       11
#define I2C_MASTER_SCL_IO       10

#define I2C_TASK_STACK_SIZE     4096
#define I2C_TASK_PRIORITY       5
#define I2C_QUEUE_LENGTH        10

#define TB_SLEEP_THRESHOLD_C    25.0f
#define TB_WAKE_THRESHOLD_C     26.0f  

#define SLEEP_I2C_POLL_MS       250    
#define ACTIVE_I2C_POLL_MS      100    

#define LOG_FILE_PATH       "/sdcard/brew_log.csv"
#define LOG_BACKUP_PATH     "/sdcard/brew_log.bak"
#define MAX_LOG_SIZE_BYTES  (5 * 1024 * 1024) 

extern void Wireless_Init(void);
extern float g_manual_target_c; 

static const char *TAG = "ARDUINO_I2C";
void queue_debug_message(uint8_t messageType, const char* message);
void set_arduino_pid_targets(float target_pressure, float target_temp);

typedef struct {
    float pressure;
    float groupHeadTemp;
    float thermoblockTemp;
    float targetPressure;   
    float targetTemp;       
    uint8_t pumpPower;
    uint8_t selectedProgram;
    uint8_t operatingMode; 
    uint8_t stateFlags;     
    uint8_t heartbeat;
    uint8_t padding;
} __attribute__((packed)) ArduinoMachineState;

#define STATE_SHOT_ACTIVE(s)   ((s).stateFlags & 0x01)
#define STATE_IS_MANUAL(s)     ((s).stateFlags & 0x02)
#define STATE_HAS_ERROR(s)     ((s).stateFlags & 0x04)

typedef struct {
    bool connected;
    unsigned long totalRequests;
    unsigned long successfulRequests;
    unsigned long lastSuccessTime;
    unsigned long consecutiveFailures;
    float successRate;
} ArduinoCommStats;

typedef enum {
    POWER_MODE_ACTIVE,
    POWER_MODE_DISPLAY_SLEEP
} power_mode_t;

static power_mode_t current_power_mode = POWER_MODE_ACTIVE;
static uint32_t last_backlight_value = 70;
static TaskHandle_t ui_task_handle = NULL;

typedef enum {
    I2C_OP_READ_SENSORS,
    I2C_OP_SEND_DEBUG,
    I2C_OP_SEND_COMMAND
} i2c_operation_type_t;

typedef struct {
    i2c_operation_type_t operation;
    uint8_t messageType;    
    uint8_t commandCode;    
    uint8_t dataLength;     
    char data[32];          
} i2c_operation_t;

ArduinoMachineState g_arduinoState = {0};
ArduinoMachineState g_lastArduinoState = {0};
ArduinoCommStats g_arduinoStats = {false, 0, 0, 0, 0, 0.0f};

static QueueHandle_t i2c_queue = NULL;
static TaskHandle_t i2c_task_handle = NULL;

typedef struct {
    uint8_t messageType;    
    uint8_t messageLength;  
    char messageData[30];   
} __attribute__((packed)) I2CDebugMessage;

bool is_arduino_connected(void);

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static bool request_arduino_state_internal(void) {
    g_arduinoStats.totalRequests++;
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, 
                                                ARDUINO_I2C_ADDRESS,
                                                (uint8_t*)&g_arduinoState, 
                                                sizeof(ArduinoMachineState),
                                                I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (err == ESP_OK) {
        g_arduinoStats.lastSuccessTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        g_arduinoStats.consecutiveFailures = 0;
        g_arduinoStats.successfulRequests++;
        g_arduinoStats.connected = true;
        g_arduinoStats.successRate = (float)g_arduinoStats.successfulRequests / g_arduinoStats.totalRequests * 100.0f;
        return true;
    } else {
        g_arduinoStats.consecutiveFailures++;
        if (g_arduinoStats.consecutiveFailures > 5) g_arduinoStats.connected = false;
        return false;
    }
}

static esp_err_t send_debug_to_arduino_internal(uint8_t messageType, const char* message) {
    I2CDebugMessage debugMsg;
    debugMsg.messageType = messageType;
    size_t msgLen = strlen(message);
    if (msgLen > 29) msgLen = 29;
    debugMsg.messageLength = msgLen;
    strncpy(debugMsg.messageData, message, msgLen);
    debugMsg.messageData[msgLen] = '\0';
    size_t payload_size = offsetof(I2CDebugMessage, messageData) + debugMsg.messageLength + 1;
    return i2c_master_write_to_device(I2C_MASTER_NUM, ARDUINO_I2C_ADDRESS, (uint8_t*)&debugMsg, payload_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t send_arduino_command_internal(uint8_t command, const void* data, size_t data_len) {
    uint8_t cmd_buffer[32];
    cmd_buffer[0] = command;
    size_t total_len = 1;
    if (data && data_len > 0 && data_len < 31) {
        memcpy(&cmd_buffer[1], data, data_len);
        total_len += data_len;
    }
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, ARDUINO_I2C_ADDRESS, cmd_buffer, total_len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return err;
}

static void check_arduino_state_changes(void) {
    bool current_active = STATE_SHOT_ACTIVE(g_arduinoState);
    bool last_active = STATE_SHOT_ACTIVE(g_lastArduinoState);
    
    if (current_active != last_active) {
        queue_debug_message(2, current_active ? "Shot started" : "Shot stopped");
    }
    g_lastArduinoState = g_arduinoState;
}

static void enter_display_sleep_mode(void) {
    if (current_power_mode == POWER_MODE_DISPLAY_SLEEP) return;
    ESP_LOGI(TAG, "Entering Sleep Mode");
    extern uint8_t LCD_Backlight;
    last_backlight_value = LCD_Backlight;
    Set_Backlight(0);
    esp_wifi_stop();
    
    esp_pm_config_t pm_config = { .max_freq_mhz = 80, .min_freq_mhz = 40, .light_sleep_enable = false };
    esp_pm_configure(&pm_config);
    
    if (ui_task_handle != NULL) vTaskSuspend(ui_task_handle);
    current_power_mode = POWER_MODE_DISPLAY_SLEEP;
}

static void wake_from_display_sleep_mode(void) {
    if (current_power_mode == POWER_MODE_ACTIVE) return;
    ESP_LOGI(TAG, "Waking Up");
    
    esp_pm_config_t pm_config = { .max_freq_mhz = 240, .min_freq_mhz = 80, .light_sleep_enable = false };
    esp_pm_configure(&pm_config);
    
    esp_wifi_start();
    if (ui_task_handle != NULL) vTaskResume(ui_task_handle);
    Set_Backlight(70);
    current_power_mode = POWER_MODE_ACTIVE;
}

static void monitor_power_mode(void) {
    // 1°C hysteresis (SLEEP=25 °C, WAKE=26 °C) prevents rapid sleep/wake cycling
    // when the thermoblock idles near the threshold.  temp_stable_count adds a
    // debounce of ~5 s (active) / ~2 s (sleeping) before any transition fires.
    static uint8_t temp_stable_count = 0;
    float tb_temp = g_arduinoState.thermoblockTemp;
    
    if (xTaskGetTickCount() * portTICK_PERIOD_MS < 15000) {
        if (current_power_mode != POWER_MODE_ACTIVE) wake_from_display_sleep_mode();
        return; 
    }

    if (current_power_mode == POWER_MODE_ACTIVE) {
        if (tb_temp < TB_SLEEP_THRESHOLD_C) {
            if (++temp_stable_count >= 5) {
                enter_display_sleep_mode();
                temp_stable_count = 0;
            }
        } else temp_stable_count = 0;
    } 
    else { 
        if (STATE_SHOT_ACTIVE(g_arduinoState)) {
             wake_from_display_sleep_mode();
             temp_stable_count = 0;
             return;
        }
        if (tb_temp >= TB_WAKE_THRESHOLD_C) {
            if (++temp_stable_count >= 2) {
                wake_from_display_sleep_mode();
                temp_stable_count = 0;
            }
        } else temp_stable_count = 0;
    }
}

static void i2c_communication_task(void *pvParameters) {
    i2c_operation_t op;
    TickType_t lastSensorRead = 0;
    
    while (1) {
        TickType_t currentTime = xTaskGetTickCount();
        TickType_t interval = (current_power_mode == POWER_MODE_DISPLAY_SLEEP) 
            ? pdMS_TO_TICKS(SLEEP_I2C_POLL_MS) : pdMS_TO_TICKS(ACTIVE_I2C_POLL_MS);
        
        if (currentTime - lastSensorRead >= interval) {
            if (request_arduino_state_internal()) {
                check_arduino_state_changes();
            }
            lastSensorRead = currentTime;
        }
        
        TickType_t qTimeout = (current_power_mode == POWER_MODE_DISPLAY_SLEEP) ? pdMS_TO_TICKS(100) : pdMS_TO_TICKS(10);
        if (xQueueReceive(i2c_queue, &op, qTimeout) == pdTRUE) {
            switch (op.operation) {
                case I2C_OP_SEND_DEBUG: send_debug_to_arduino_internal(op.messageType, op.data); break;
                case I2C_OP_SEND_COMMAND: send_arduino_command_internal(op.commandCode, op.data, op.dataLength); break;
                default: break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void queue_debug_message(uint8_t messageType, const char* message) {
    if (i2c_queue == NULL) return;
    i2c_operation_t op = { .operation = I2C_OP_SEND_DEBUG, .messageType = messageType, .dataLength = 0 };
    strncpy(op.data, message, sizeof(op.data) - 1);
    xQueueSend(i2c_queue, &op, 0);
}

void queue_arduino_command(uint8_t command, const void* data, size_t data_len) {
    if (i2c_queue == NULL) return;
    i2c_operation_t op = { .operation = I2C_OP_SEND_COMMAND, .commandCode = command, .dataLength = data_len };
    if (data && data_len > 0) memcpy(op.data, data, data_len);
    xQueueSend(i2c_queue, &op, pdMS_TO_TICKS(50));
}

static void background_drivers_task(void *parameter) {
    while(1) {
        PCF85063_Loop();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

static void drivers_init(void) {
    EXIO_Init();
    Flash_Searching();
    PCF85063_Init();
    xTaskCreatePinnedToCore(background_drivers_task, "background_drivers", 4096, NULL, 3, NULL, 0);
}

void log_data_to_sd(void) {
    static uint32_t last_log_time = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (now - last_log_time < 1000) return;
    last_log_time = now;

    if (!g_arduinoStats.connected) return;

    struct stat st;
    if (stat(LOG_FILE_PATH, &st) == 0 && st.st_size >= MAX_LOG_SIZE_BYTES) {
        unlink(LOG_BACKUP_PATH); 
        rename(LOG_FILE_PATH, LOG_BACKUP_PATH);
    }

    bool write_header = (stat(LOG_FILE_PATH, &st) != 0);
    float delta_t = g_arduinoState.groupHeadTemp - g_arduinoState.thermoblockTemp;
    bool is_active = STATE_SHOT_ACTIVE(g_arduinoState);

    char log_buffer[128];
    snprintf(log_buffer, sizeof(log_buffer), 
             "%lu,%d,%d,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f\n",
             now,
             g_arduinoState.operatingMode,
             is_active,
             g_arduinoState.pressure,
             g_arduinoState.groupHeadTemp,
             g_arduinoState.thermoblockTemp,
             delta_t,
             g_arduinoState.pumpPower,
             g_arduinoState.targetPressure,
             g_arduinoState.targetTemp
    );

    FILE *f = fopen(LOG_FILE_PATH, "a");
    if (f != NULL) {
        if (write_header) {
            fprintf(f, "Time,Mode,Active,Press,GH,TB,DeltaT,Power,TgtP,TgtT\n");
        }
        fprintf(f, "%s", log_buffer);
        fclose(f);
    }
}

void process_active_shot(void) {
    // 1. MANUAL MODE LOGIC
    if (g_arduinoState.operatingMode == 1) { // MODE_MANUAL
        static uint32_t last_manual_send = 0;
        if (xTaskGetTickCount() - last_manual_send > pdMS_TO_TICKS(1000)) {
            set_arduino_pid_targets(0.0f, g_manual_target_c); 
            last_manual_send = xTaskGetTickCount();
        }
        return; 
    }

    // 2. AUTO MODE LOGIC
    static int64_t shot_start_time = 0;
    static bool shot_in_progress = false;
    static bool shot_completed_latch = false;
    static uint32_t shot_activation_grace_period = 0; // FIX: Grace Timer

    bool arduino_active = STATE_SHOT_ACTIVE(g_arduinoState);
    
    // === RESET LOGIC ===
    // If we are running, but Arduino goes inactive, normally we stop.
    // FIX: Don't stop if we are in the "Grace Period" (waiting for Arduino to wake up)
    if (shot_in_progress) {
        bool grace_period_active = (xTaskGetTickCount() - shot_activation_grace_period < pdMS_TO_TICKS(2000));
        
        if (!arduino_active && !grace_period_active) {
             shot_in_progress = false;
             // We don't clear the latch here; we fall through to latch reset below
        }
    }

    // === LATCH RESET ===
    // Prevents restarting until pressure drops back to 0
    if (shot_completed_latch) {
        if (!arduino_active && g_arduinoState.pressure < 0.5f) {
            shot_completed_latch = false;
            ESP_LOGI("SHOT", "System Idle - Re-armed");
        }
        return; // Block restarts while latched
    }

    // === TRIGGER LOGIC ===
    if (!shot_in_progress && !shot_completed_latch) {
        // Start if Arduino Active OR Pressure Spike > 1.5 bar
        if (arduino_active || (g_arduinoState.pressure > 1.5f)) {
            // Safety: Boot lockout (5s)
            if (esp_timer_get_time() < 5000000) return;

            shot_start_time = esp_timer_get_time();
            shot_in_progress = true;
            
            // FIX: Set Grace Period timestamp to prevent immediate reset
            shot_activation_grace_period = xTaskGetTickCount();
            
            ESP_LOGI("SHOT", "Auto Shot Detected. Profile: %ld", (long)g_arduinoState.selectedProgram);
        }
    }

    // === EXECUTION LOGIC ===
    if (shot_in_progress) {
        float elapsed_s = (esp_timer_get_time() - shot_start_time) / 1000000.0f;
        int profile_idx = g_arduinoState.selectedProgram;
        
        const brew_profile_t* p = get_profile(profile_idx);
        
        // CHECK FINISH LINE
        if (p && elapsed_s >= p->duration_seconds) {
            ESP_LOGI("SHOT", "Profile Complete (Duration: %lu s). Stopping.", (unsigned long)p->duration_seconds);
            
            // Send Stop Command
            set_arduino_pid_targets(0.0f, p->target_temperature_c);
            
            shot_completed_latch = true; 
            shot_in_progress = false;
            return;
        }

        // RUN PROFILE
        float target_p = get_target_pressure(profile_idx, elapsed_s);
        float target_t = get_target_temperature(profile_idx, elapsed_s);
        set_arduino_pid_targets(target_p, target_t);
    }
}
void trigger_arduino_power_button(void) {
    queue_arduino_command(0x20, NULL, 0);
}

// Helper to check if machine needs to wake up
// Helper to check if machine needs to wake up
static void check_schedule_wake(void) {
    // 1. Get Current Time (Local)
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // DEBUG: Print current state every time we check (every 10s)
    // This confirms the loop is actually running.
    // ESP_LOGI("SCHED", "Checking Schedule... Day: %d, Time: %02d:%02d", timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min);

    // 2. Get Schedule for Today
    const schedule_entry_t* entry = get_schedule_for_day(timeinfo.tm_wday);
    
    if (!entry) {
        ESP_LOGW("SCHED", "No schedule entry found for Day %d", timeinfo.tm_wday);
        return;
    }
    
    if (!entry->enabled) {
        // ESP_LOGD("SCHED", "Schedule disabled for Day %d", timeinfo.tm_wday);
        return;
    }

    // 3. Calculate "Minutes from Midnight"
    int current_min_day = (timeinfo.tm_hour * 60) + timeinfo.tm_min;
    int start_min_day   = (entry->hour * 60) + entry->minute;
    
    // 4. Window Logic
    int diff = current_min_day - start_min_day;
    static int last_trigger_day = -1;

    // Debug the math if we are close
    if (abs(diff) < 60) {
        ESP_LOGI("SCHED", "Window Check: Start=%d, Now=%d, Diff=%d (Need 0-15)", start_min_day, current_min_day, diff);
    }

    if (diff >= 0 && diff < 15) {
        if (last_trigger_day != timeinfo.tm_wday) {
            
            // SAFETY CHECK LOGGING
            float current_temp = g_arduinoState.thermoblockTemp;
            ESP_LOGI("SCHED", "Inside Window! Temp: %.1f C (Limit: 40.0 C)", current_temp);
            
            if (current_temp < 40.0f) {
                ESP_LOGI("SCHED", "TRIGGERING WAKE! (Pulse Power Button)");
                trigger_arduino_power_button(); 
                last_trigger_day = timeinfo.tm_wday; 
            } else {
                ESP_LOGW("SCHED", "Skipped Wake: Machine is already Hot (%.1f C)", current_temp);
                last_trigger_day = timeinfo.tm_wday; // Mark done so we don't keep checking
            }
        } else {
            // Already triggered today
            // ESP_LOGD("SCHED", "Already triggered for today");
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "ESP32 Master Starting - V46.1 Schedule Fix");
    
    i2c_queue = xQueueCreate(10, sizeof(i2c_operation_t));
    i2c_master_init();
    xTaskCreatePinnedToCore(i2c_communication_task, "i2c_comm", 4096, NULL, 5, &i2c_task_handle, 0);

    esp_pm_config_t pm_config = { .max_freq_mhz = 240, .min_freq_mhz = 80, .light_sleep_enable = false };
    esp_pm_configure(&pm_config);

    drivers_init(); 
    SD_Init();
    vTaskDelay(pdMS_TO_TICKS(500)); 
    
    profile_manager_init(); 
    Wireless_Init(); 
    time_init();     
    
    LCD_Init();
    LVGL_Init();
    Lvgl_Coffee_UI_Init();
    
    ESP_LOGI("MAIN", "Forcing Backlight ON");
    Set_Backlight(70); 
    
    ESP_LOGI("MAIN", "System init complete");
    
    static uint32_t loop_counter = 0;
    while (1) {
        // Run periodic checks (every 500ms)
        if (++loop_counter % 50 == 0) { 
            monitor_power_mode(); 
            
            // === FIX: Call the Schedule Check (Every 10 seconds) ===
            if (loop_counter % 1000 == 0) {
                check_schedule_wake();
            }
        }

        if (current_power_mode == POWER_MODE_ACTIVE) {
            
            // 1. Handle Shots
            process_active_shot();
            
            // 2. Handle Idle Preheat (Auto Mode Only)
            if (g_arduinoState.operatingMode == 2 && !STATE_SHOT_ACTIVE(g_arduinoState)) {
                if (loop_counter % 100 == 0) { // Send every 1s
                    int idx = g_arduinoState.selectedProgram;
                    const brew_profile_t* p = get_profile(idx);
                    // Default to 93.0 if profile invalid
                    float preheat = p ? p->preheat_temperature_c : 93.0f;
                    set_arduino_pid_targets(0.0f, preheat);
                }
            }

            log_data_to_sd();
            lv_timer_handler();
            Lvgl_Coffee_UI_Loop();
        }
        
        // HEARTBEAT LOG (Every 2s)
        if (loop_counter % 200 == 0) {
             if (is_arduino_connected()) {
                 char mode_str[64];
                 float active_target_temp = g_arduinoState.targetTemp; 
                 
                 if (g_arduinoState.operatingMode == 1) {
                     snprintf(mode_str, sizeof(mode_str), "MANUAL");
                 } 
                 else if (g_arduinoState.operatingMode == 2) {
                     int idx = g_arduinoState.selectedProgram;
                     const brew_profile_t* p = get_profile(idx);
                     const char* p_name = p ? p->name : "Unknown";
                     
                     if (STATE_SHOT_ACTIVE(g_arduinoState)) {
                         snprintf(mode_str, sizeof(mode_str), "AUTO-BREW (%s)", p_name);
                     } else {
                         float intended_preheat = p ? p->preheat_temperature_c : 93.0f;
                         snprintf(mode_str, sizeof(mode_str), "AUTO-IDLE (%s) [Cfg:%.1f]", p_name, intended_preheat);
                     }
                 } 
                 else {
                     snprintf(mode_str, sizeof(mode_str), "BYPASS");
                 }

                 ESP_LOGI("STATUS", "%-30s | Press: %4.1f / %4.1f | Temp: %4.1f / %4.1f | Pwr: %d%%", 
                     mode_str,
                     g_arduinoState.pressure,
                     g_arduinoState.targetPressure,
                     g_arduinoState.thermoblockTemp,
                     active_target_temp,
                     g_arduinoState.pumpPower
                 );
             } else {
                 ESP_LOGW("STATUS", "Arduino Disconnected");
             }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

const ArduinoMachineState* get_arduino_state(void) { return &g_arduinoState; }
const ArduinoCommStats* get_arduino_stats(void) { return &g_arduinoStats; }
bool is_arduino_connected(void) { return g_arduinoStats.connected; }

void set_arduino_pid_targets(float target_pressure, float target_temp) {
    struct { float p; float t; } targets = {target_pressure, target_temp};
    queue_arduino_command(0x10, &targets, sizeof(targets));
}

// =============================================================================
