/**
 * @file Wireless.c
 * @brief Wi-Fi, BLE, and Configuration Manager
 * @version V46 - Restored Logic
 * @date 2025-11-29
 * @details
 * This module performs three critical startup tasks:
 * 1. **Config Loader:** Mounts the SD card and parses `/sdcard/config.json` to get
 * Wi-Fi credentials and the user's preferred Manual Mode base temp/bias.
 * 2. **Network Manager:** Connects to Wi-Fi and syncs NTP time (Central US).
 * 3. **BLE Scanner:** background scanner (functionality preserved from legacy code).
 * * @note
 * Defines the global `g_manual_target_c` used by the UI and Main logic.
 */

#include "Wireless.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_sntp.h"
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "cJSON.h" 

static const char *TAG = "WIFI";

// =============================================================================
// GLOBAL STATE & CREDENTIALS
// =============================================================================

// Global buffers for credentials (defaults overwritten by SD card)
char wifi_ssid[33] = "DEFAULT_SSID";
char wifi_password[64] = "DEFAULT_PASS";

/**
 * @brief Global Manual Mode Target Temperature (Celsius).
 * @details Loaded from config.json (base + bias). Used by main.c for manual shots.
 */
float g_manual_target_c = 93.0f; 

// BLE & Scan State Variables
uint16_t BLE_NUM = 0;
uint16_t WIFI_NUM = 0;        
bool Scan_finish = 0;
bool WiFi_Scan_Finish = 0;    
bool BLE_Scan_Finish = 0;

// External Helper from time_sync.c
extern void update_rtc_from_system(void);

// Forward Declarations
void WIFI_Init(void *arg);
uint16_t WIFI_Scan(void);
void BLE_Init(void *arg);
uint16_t BLE_Scan(void);

// =============================================================================
// CONFIGURATION LOADER
// =============================================================================

/**
 * @brief loads `config.json` from SD card.
 * @details Parses Wi-Fi credentials and Manual Mode temp settings.
 * If file is missing/invalid, defaults are used.
 */
void load_wifi_config(void)
{
    ESP_LOGI(TAG, "Attempting to load config from /sdcard/config.json");
    
    FILE *f = fopen("/sdcard/config.json", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open config.json. Using defaults.");
        return;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    long length = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (length <= 0) {
        ESP_LOGE(TAG, "Config file empty");
        fclose(f);
        return;
    }

    // Allocate buffer
    char *buffer = (char *)malloc(length + 1);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for config");
        fclose(f);
        return;
    }

    // Read file
    fread(buffer, 1, length, f);
    buffer[length] = '\0';
    fclose(f);

    // Parse JSON
    cJSON *root = cJSON_Parse(buffer);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON config");
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) ESP_LOGE(TAG, "Error before: %s", error_ptr);
        free(buffer);
        return;
    }

    // Navigate to "wifi" object
    cJSON *wifi = cJSON_GetObjectItemCaseSensitive(root, "wifi");
    if (wifi) {
        cJSON *ssid = cJSON_GetObjectItemCaseSensitive(wifi, "ssid");
        cJSON *pass = cJSON_GetObjectItemCaseSensitive(wifi, "password");

        if (cJSON_IsString(ssid) && (ssid->valuestring != NULL)) {
            strncpy(wifi_ssid, ssid->valuestring, sizeof(wifi_ssid) - 1);
            wifi_ssid[sizeof(wifi_ssid) - 1] = '\0'; // Ensure null termination
            ESP_LOGI(TAG, "Loaded SSID: %s", wifi_ssid);
        }

        if (cJSON_IsString(pass) && (pass->valuestring != NULL)) {
            strncpy(wifi_password, pass->valuestring, sizeof(wifi_password) - 1);
            wifi_password[sizeof(wifi_password) - 1] = '\0';
            ESP_LOGI(TAG, "Loaded Password: [HIDDEN]");
        }
    } else {
        ESP_LOGW(TAG, "No 'wifi' object found in config.json");
    }
    
    // === LOAD MANUAL CONFIG ===
    cJSON *manual = cJSON_GetObjectItemCaseSensitive(root, "manual_mode");
    if (manual) {
        cJSON *base = cJSON_GetObjectItemCaseSensitive(manual, "base_temp_c");
        cJSON *bias = cJSON_GetObjectItemCaseSensitive(manual, "temp_bias_f");
        
        float base_c = (cJSON_IsNumber(base)) ? base->valuedouble : 93.0f;
        float bias_f = (cJSON_IsNumber(bias)) ? bias->valuedouble : 0.0f;
        
        // Convert Bias F -> C and add to Base
        g_manual_target_c = base_c + (bias_f * 5.0f / 9.0f);
        ESP_LOGI(TAG, "Manual Config Loaded: Target %.1fC (Base %.1f + Bias %.1fF)", 
                 g_manual_target_c, base_c, bias_f);
    }

    // Cleanup
    cJSON_Delete(root);
    free(buffer);
}

// =============================================================================
// TIME SYNCHRONIZATION
// =============================================================================

/**
 * @brief Callback for NTP Sync events.
 * @details Sets the timezone to Central US (Chicago) and pushes the time to the hardware RTC.
 */
void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "NTP Synchronization Complete");
    
    // 1. Set Timezone to Central US (Chicago)
    setenv("TZ", "CST6CDT,M3.2.0,M11.1.0", 1);
    tzset();
    
    // 2. Print Time
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "CURRENT LOCAL TIME (Central): %s", strftime_buf);
    
    // 3. Update the Hardware RTC (Critical for schedule)
    update_rtc_from_system();
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

// =============================================================================
// WIFI HANDLING
// =============================================================================

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi Started, attempting to connect...");
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi Disconnected. Retrying...");
        esp_wifi_connect();
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        
        // Start Time Sync once we have internet
        initialize_sntp();
    }
}

/**
 * @brief Main Wi-Fi Initialization Task.
 * @details Loads config, sets up Netif, and registers handlers.
 * Note: Deletes itself after init is complete.
 */
void WIFI_Init(void *arg)
{
    // 1. LOAD CONFIG FIRST
    load_wifi_config();

    ESP_LOGI(TAG, "Initializing WiFi Station Mode");

    esp_netif_init();                                                         
    esp_event_loop_create_default();                                          
    esp_netif_create_default_wifi_sta();                                      
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();                 
    esp_wifi_init(&cfg);    

    // Register Event Handlers
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    // Configure WiFi Credentials using the globals loaded from JSON
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    
    // Copy credentials from global buffers
    strncpy((char*)wifi_config.sta.ssid, wifi_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, wifi_password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start()); 

    ESP_LOGI(TAG, "WiFi Init Complete.");
    
    vTaskDelete(NULL); 
}

// =============================================================================
// BLE HANDLING
// =============================================================================

#define GATTC_TAG "GATTC_TAG"
#define SCAN_DURATION 5   
#define MAX_DISCOVERED_DEVICES 100 

typedef struct {
    uint8_t address[6];
    bool is_valid;
} discovered_device_t;

static discovered_device_t discovered_devices[MAX_DISCOVERED_DEVICES];
static size_t num_discovered_devices = 0;
static size_t num_devices_with_name = 0; 

static bool is_device_discovered(const uint8_t *addr) {
    for (size_t i = 0; i < num_discovered_devices; i++) {
        if (memcmp(discovered_devices[i].address, addr, 6) == 0) {
            return true;
        }
    }
    return false;
}

static void add_device_to_list(const uint8_t *addr) {
    if (num_discovered_devices < MAX_DISCOVERED_DEVICES) {
        memcpy(discovered_devices[num_discovered_devices].address, addr, 6);
        discovered_devices[num_discovered_devices].is_valid = true;
        num_discovered_devices++;
    }
}

static bool extract_device_name(const uint8_t *adv_data, uint8_t adv_data_len, char *device_name, size_t max_name_len) {
    size_t offset = 0;
    while (offset < adv_data_len) {
        if (adv_data[offset] == 0) break; 

        uint8_t length = adv_data[offset];
        if (length == 0 || offset + length > adv_data_len) break; 

        uint8_t type = adv_data[offset + 1];
        if (type == ESP_BLE_AD_TYPE_NAME_CMPL || type == ESP_BLE_AD_TYPE_NAME_SHORT) {
            if (length > 1 && length - 1 < max_name_len) {
                memcpy(device_name, &adv_data[offset + 2], length - 1);
                device_name[length - 1] = '\0'; 
                return true;
            } else {
                return false;
            }
        }
        offset += length + 1;
    }
    return false;
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    static char device_name[100]; 

    switch (event) {
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                if (!is_device_discovered(param->scan_rst.bda)) {
                    add_device_to_list(param->scan_rst.bda);
                    BLE_NUM++; 

                    if (extract_device_name(param->scan_rst.ble_adv, param->scan_rst.adv_data_len, device_name, sizeof(device_name))) {
                        num_devices_with_name++;
                    }
                }
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            ESP_LOGI(GATTC_TAG, "Scan complete. Total devices found: %d (with names: %d)", BLE_NUM, num_devices_with_name);
            break;
        default:
            break;
    }
}

void BLE_Init(void *arg)
{
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_bt_controller_init(&bt_cfg);                                                 
    if (ret) {
        printf("%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));         
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);                                                 
    if (ret) {
        printf("%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));             
        return;
    }
    ret = esp_bluedroid_init();                                                                      
    if (ret) {
        printf("%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));                
        return;
    }
    ret = esp_bluedroid_enable();                                                                    
    if (ret) {
        printf("%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));              
        return;
    }

    // Register callback function to GAP module
    ret = esp_ble_gap_register_callback(esp_gap_cb);                                                 
    if (ret){
        printf("%s gap register error, error code = %x\n", __func__, ret);                       
        return;
    }
    BLE_Scan();
    
    vTaskDelete(NULL);
}

uint16_t BLE_Scan(void)
{
    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_RPA_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,     
        .scan_window = 0x30,        
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };
    ESP_ERROR_CHECK(esp_ble_gap_set_scan_params(&scan_params));

    printf("Starting BLE scan...\n");
    ESP_ERROR_CHECK(esp_ble_gap_start_scanning(SCAN_DURATION));
    
    vTaskDelay(SCAN_DURATION * 1000 / portTICK_PERIOD_MS);
    
    printf("Stopping BLE scan...\n");
    ESP_ERROR_CHECK(esp_ble_gap_stop_scanning()); 
    BLE_Scan_Finish = 1;
    Scan_finish = 1;
    return BLE_NUM;
}

// =============================================================================
// PUBLIC API
// =============================================================================

void Wireless_Init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    // Create WiFi Task (Core 1)
    // Increased stack size to 6144 to handle JSON parsing overhead
    xTaskCreatePinnedToCore(WIFI_Init, "WIFI task", 6144, NULL, 1, NULL, 0);
        
    // Create BLE Task (Core 0)
    xTaskCreatePinnedToCore(BLE_Init, "BLE task", 4096, NULL, 2, NULL, 0);
}