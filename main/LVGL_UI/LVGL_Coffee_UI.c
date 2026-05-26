/**
 * @file LVGL_Coffee_UI.c
 * @brief Breville Profusion LVGL UI — display layer for the 360×360 round LCD.
 *
 * Converts raw ArduinoMachineState data (polled via I2C in main.c) into visible
 * labels and a progress bar.  Uses string-lookup tables and change-detection to
 * minimise LVGL redraw calls.
 *
 * All pixel coordinates are hardcoded for the 360×360 round ST77916 display.
 * LVGL's alignment helpers cannot account for the circular bezel, so absolute
 * offsets are tuned manually per widget.
 *
 * queue_debug_message() calls in this file send telemetry strings to the Arduino
 * over I2C — they are not printf/UART debug output.
 */

#include "LVGL_Coffee_UI.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_timer.h" // Required for high-res shot timer
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// =============================================================================
// CONFIGURATION & MACROS
// =============================================================================
static const char *TAG = "COFFEE_UI";

// Color Palette definition for consistent styling
#define COLOR_PRIMARY       lv_color_hex(0xFFFFFF)  // White
#define COLOR_ACCENT        lv_color_hex(0x00AAFF)  // Light blue accent
#define COLOR_PRESSURE      lv_color_hex(0x444444)  // Dark grey for pressure
#define COLOR_TEMPERATURE   lv_color_hex(0x666666)  // Medium grey for temperature
#define COLOR_SUCCESS_GREEN lv_color_hex(0x00AA00)  // Green for status
#define COLOR_ERROR_RED     lv_color_hex(0xFF0000)  // Red for errors
#define COLOR_BLACK         lv_color_hex(0x000000)  // Black text
#define COLOR_LIGHT_GREY    lv_color_hex(0xAAAAAA)  // Passive elements
#define COLOR_MEDIUM_GREY   lv_color_hex(0x888888)  // Dividers/backgrounds
#define COLOR_DARK_GREY     lv_color_hex(0x444444)  // Headers

// =============================================================================
// EXTERNAL DATA STRUCTURES & DEPENDENCIES
// =============================================================================

// Matches Arduino struct byte-for-byte
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

typedef enum {
    MODE_BYPASS = 0,    
    MODE_MANUAL = 1,
    MODE_AUTO = 2
} operating_mode_t;

// External Getters/Helpers from main.c
extern void queue_debug_message(uint8_t messageType, const char* message);
extern const ArduinoMachineState* get_arduino_state(void);
extern bool is_arduino_connected(void);
extern const char* get_profile_name(uint8_t index); // From profile_manager.c

// Image Asset
LV_IMG_DECLARE(background);

// =============================================================================
// UI OBJECT REFERENCES
// =============================================================================
// Pointers to active LVGL widgets to allow updates later
static lv_obj_t *main_screen = NULL;
static lv_obj_t *pressure_label = NULL;
static lv_obj_t *temp_gh_label = NULL;
static lv_obj_t *temp_tb_label = NULL;
static lv_obj_t *timer_label = NULL;
static lv_obj_t *mode_label = NULL;
static lv_obj_t *status_label = NULL;
static lv_obj_t *profile_label = NULL;
static lv_obj_t *progress_bar = NULL;
static lv_obj_t *connection_status_label = NULL;
static lv_obj_t *target_temp_label = NULL;

// =============================================================================
// OPTIMIZATION TABLES & BUFFERS
// =============================================================================

// Pre-formatted pressure strings. Avoiding snprintf in the fast loop saves IRAM and CPU cycles.
static const char* PRESSURE_FORMAT_STRINGS[] = {
    "0.0 bar", "0.1 bar", "0.2 bar", "0.3 bar", "0.4 bar",
    "0.5 bar", "0.6 bar", "0.7 bar", "0.8 bar", "0.9 bar",
    "1.0 bar", "1.1 bar", "1.2 bar", "1.3 bar", "1.4 bar",
    "1.5 bar", "1.6 bar", "1.7 bar", "1.8 bar", "1.9 bar",
    "2.0 bar", "2.1 bar", "2.2 bar", "2.3 bar", "2.4 bar",
    "2.5 bar", "2.6 bar", "2.7 bar", "2.8 bar", "2.9 bar",
    "3.0 bar", "3.1 bar", "3.2 bar", "3.3 bar", "3.4 bar",
    "3.5 bar", "3.6 bar", "3.7 bar", "3.8 bar", "3.9 bar",
    "4.0 bar", "4.1 bar", "4.2 bar", "4.3 bar", "4.4 bar",
    "4.5 bar", "4.6 bar", "4.7 bar", "4.8 bar", "4.9 bar",
    "5.0 bar", "5.1 bar", "5.2 bar", "5.3 bar", "5.4 bar",
    "5.5 bar", "5.6 bar", "5.7 bar", "5.8 bar", "5.9 bar",
    "6.0 bar", "6.1 bar", "6.2 bar", "6.3 bar", "6.4 bar",
    "6.5 bar", "6.6 bar", "6.7 bar", "6.8 bar", "6.9 bar",
    "7.0 bar", "7.1 bar", "7.2 bar", "7.3 bar", "7.4 bar",
    "7.5 bar", "7.6 bar", "7.7 bar", "7.8 bar", "7.9 bar",
    "8.0 bar", "8.1 bar", "8.2 bar", "8.3 bar", "8.4 bar",
    "8.5 bar", "8.6 bar", "8.7 bar", "8.8 bar", "8.9 bar",
    "9.0 bar", "9.1 bar", "9.2 bar", "9.3 bar", "9.4 bar",
    "9.5 bar", "9.6 bar", "9.7 bar", "9.8 bar", "9.9 bar",
    "10.0 bar", "10.1 bar", "10.2 bar", "10.3 bar", "10.4 bar",
    "10.5 bar", "10.6 bar", "10.7 bar", "10.8 bar", "10.9 bar",
    "11.0 bar", "11.1 bar", "11.2 bar", "11.3 bar", "11.4 bar",
    "11.5 bar", "11.6 bar", "11.7 bar", "11.8 bar", "11.9 bar",
    "12.0 bar"
};

static char temp_display_buffer_gh[16];
static char temp_display_buffer_tb[16];
static char timer_display_buffer[16];

// =============================================================================
// LOCAL STATE MANAGEMENT
// =============================================================================

/**
 * @brief Local copy of the machine state with caching logic.
 * @details We store "last_*" values to detect changes. We only call LVGL update functions
 * when the value actually changes, which significantly improves rendering performance.
 */
typedef struct {
    float pressure;          // 0-12 bar
    float temp_grouphead;    // C
    float temp_thermoblock;  // C
    bool shot_active;
    operating_mode_t operating_mode; 
    int selected_profile;    
    int shot_time_seconds;   // Calculated internally
    bool arduino_connected;
    uint8_t heartbeat;
    
    // Change Detection Cache
    float last_pressure;
    float last_temp_gh;
    float last_temp_tb;
    int last_timer;
    operating_mode_t last_operating_mode;
    int last_profile;
    bool last_connected;
} coffee_data_t;

static coffee_data_t coffee_data = {
    .pressure = 0.0f,
    .temp_grouphead = 0.0f,
    .temp_thermoblock = 0.0f,
    .shot_active = false,
    .operating_mode = MODE_AUTO,
    .selected_profile = 0,
    .shot_time_seconds = 0,
    .arduino_connected = false,
    .heartbeat = 0,
    // Cache initialized to impossible values to force first update
    .last_pressure = -1.0f,
    .last_temp_gh = -1000.0f,
    .last_temp_tb = -1000.0f,
    .last_timer = -1,
    .last_operating_mode = 255,
    .last_profile = -1,
    .last_connected = false
};

// =============================================================================
// FORMATTING HELPERS
// =============================================================================

/**
 * @brief Returns a static string pointer for the pressure value.
 * @details Uses a lookup table instead of `snprintf` to avoid dynamic allocation
 * and heavy formatting overhead in the fast refresh loop.
 * @param pressure Pressure in bars (0.0 - 12.0)
 * @return const char* Pointer to the formatted string (e.g. "9.0 bar")
 */
static const char* get_pressure_string(float pressure) {
    if (pressure < 0.0f || pressure > 12.0f) return "-- bar";
    // Map 0.0-12.0 -> 0-120 index
    int index = (int)(pressure * 10.0f);
    if (index >= 0 && index < (sizeof(PRESSURE_FORMAT_STRINGS) / sizeof(PRESSURE_FORMAT_STRINGS[0]))) {
        return PRESSURE_FORMAT_STRINGS[index];
    }
    return "ERR bar";
}

/**
 * @brief High-speed integer-based formatting for Group Head temperature.
 * @details Avoids floating point `printf` to save cycles. Handles invalid/disconnected sensors (-100.0f).
 * @note Writes result into global `temp_display_buffer_gh`.
 * @param temp Temperature in Celsius
 */
static void format_temperature_gh(float temp) {
    if (temp > -50.0f && temp < 120.0f) {
        int temp_int = (int)temp;
        int temp_dec = (int)((temp - temp_int) * 10) % 10;
        
        strcpy(temp_display_buffer_gh, "GH: ");
        // Manual integer to string conversion for speed
        if (temp_int >= 100) {
            temp_display_buffer_gh[4] = '0' + (temp_int / 100);
            temp_display_buffer_gh[5] = '0' + ((temp_int / 10) % 10);
            temp_display_buffer_gh[6] = '0' + (temp_int % 10);
            temp_display_buffer_gh[7] = '.';
            temp_display_buffer_gh[8] = '0' + temp_dec;
            temp_display_buffer_gh[9] = '\xB0'; // Degree symbol
            temp_display_buffer_gh[10] = 'C';
            temp_display_buffer_gh[11] = '\0';
        } else if (temp_int >= 10) {
            temp_display_buffer_gh[4] = '0' + (temp_int / 10);
            temp_display_buffer_gh[5] = '0' + (temp_int % 10);
            temp_display_buffer_gh[6] = '.';
            temp_display_buffer_gh[7] = '0' + temp_dec;
            temp_display_buffer_gh[8] = '\xB0';
            temp_display_buffer_gh[9] = 'C';
            temp_display_buffer_gh[10] = '\0';
        } else {
            temp_display_buffer_gh[4] = '0' + temp_int;
            temp_display_buffer_gh[5] = '.';
            temp_display_buffer_gh[6] = '0' + temp_dec;
            temp_display_buffer_gh[7] = '\xB0';
            temp_display_buffer_gh[8] = 'C';
            temp_display_buffer_gh[9] = '\0';
        }
    } else {
        strcpy(temp_display_buffer_gh, "GH: No Sensor");
    }
}

/**
 * @brief High-speed integer-based formatting for Thermoblock temperature.
 * @details Avoids floating point `printf` to save cycles. Handles invalid/disconnected sensors (-100.0f).
 * @note Writes result into global `temp_display_buffer_tb`.
 * @param temp Temperature in Celsius
 */
static void format_temperature_tb(float temp) {
    if (temp > -50.0f && temp < 120.0f) {
        int temp_int = (int)temp;
        int temp_dec = (int)((temp - temp_int) * 10) % 10;
        
        strcpy(temp_display_buffer_tb, "TB: ");
        // Same optimization logic as GH
        if (temp_int >= 100) {
            temp_display_buffer_tb[4] = '0' + (temp_int / 100);
            temp_display_buffer_tb[5] = '0' + ((temp_int / 10) % 10);
            temp_display_buffer_tb[6] = '0' + (temp_int % 10);
            temp_display_buffer_tb[7] = '.';
            temp_display_buffer_tb[8] = '0' + temp_dec;
            temp_display_buffer_tb[9] = '\xB0';
            temp_display_buffer_tb[10] = 'C';
            temp_display_buffer_tb[11] = '\0';
        } else if (temp_int >= 10) {
            temp_display_buffer_tb[4] = '0' + (temp_int / 10);
            temp_display_buffer_tb[5] = '0' + (temp_int % 10);
            temp_display_buffer_tb[6] = '.';
            temp_display_buffer_tb[7] = '0' + temp_dec;
            temp_display_buffer_tb[8] = '\xB0';
            temp_display_buffer_tb[9] = 'C';
            temp_display_buffer_tb[10] = '\0';
        } else {
            temp_display_buffer_tb[4] = '0' + temp_int;
            temp_display_buffer_tb[5] = '.';
            temp_display_buffer_tb[6] = '0' + temp_dec;
            temp_display_buffer_tb[7] = '\xB0';
            temp_display_buffer_tb[8] = 'C';
            temp_display_buffer_tb[9] = '\0';
        }
    } else {
        strcpy(temp_display_buffer_tb, "TB: No Sensor");
    }
}

/**
 * @brief Formats seconds into "MM:SS" string.
 * @note Writes result into global `timer_display_buffer`.
 * @param seconds Total seconds to display.
 */
static void format_timer(int seconds) {
    int minutes = seconds / 60;
    int secs = seconds % 60;
    
    strcpy(timer_display_buffer, "Timer: ");
    timer_display_buffer[7] = '0' + (minutes / 10);
    timer_display_buffer[8] = '0' + (minutes % 10);
    timer_display_buffer[9] = ':';
    timer_display_buffer[10] = '0' + (secs / 10);
    timer_display_buffer[11] = '0' + (secs % 10);
    timer_display_buffer[12] = '\0';
}

/**
 * @brief Determines system thermal status (Ready/Warming) based on temp deltas.
 * @details Checks if the Group Head is sufficiently heated relative to the Thermoblock.
 * Applies hysteresis to prevent state flapping at the threshold.
 * @param buffer Buffer to write the status text ("Ready", "Warming").
 * @param len Length of the buffer.
 * @param color Pointer to an `lv_color_t` to set the text color.
 */
static void get_thermal_status_text(char* buffer, size_t len, lv_color_t* color) {
    if (!coffee_data.arduino_connected) {
        strncpy(buffer, "Disconnected", len);
        *color = COLOR_ERROR_RED;
        return;
    }
    
    float delta_t = coffee_data.temp_grouphead - coffee_data.temp_thermoblock;
    static bool is_ready_state = false;
    
    // Hysteresis Logic: Warming < -5.5 < Band < -4.5 < Ready
    if (is_ready_state) {
        if (delta_t < -5.5f) is_ready_state = false;
    } else {
        if (delta_t > -4.5f) is_ready_state = true;
    }
    
    if (is_ready_state) {
        strncpy(buffer, "Ready", len);
        *color = COLOR_MEDIUM_GREY;
    } else {
        strncpy(buffer, "Warming", len);
        *color = COLOR_MEDIUM_GREY;
    }
}

// =============================================================================
// DATA SYNC & TIMING LOGIC
// =============================================================================

/**
 * @brief Synchronizes local `coffee_data` with the global `g_arduinoState`.
 * @details Also calculates the "Effective Timer". The timer logic is smart:
 * - It only increments when pressure is > 1.0 bar (skipping Pre-Infusion dwell).
 * - It automatically resets to 0 when a new pressure rise is detected.
 */
static void update_coffee_data_from_arduino(void) {
    const ArduinoMachineState* state = get_arduino_state();
    if (!state) return;

    coffee_data.pressure = state->pressure;
    coffee_data.temp_grouphead = state->groupHeadTemp;
    coffee_data.temp_thermoblock = state->thermoblockTemp;
    coffee_data.shot_active = (state->stateFlags & 0x01); 
    coffee_data.selected_profile = state->selectedProgram;
    coffee_data.operating_mode = (operating_mode_t)state->operatingMode;
    coffee_data.arduino_connected = is_arduino_connected();
    coffee_data.heartbeat = state->heartbeat;
    
    // === SMART TIMER LOGIC ===
    static int64_t last_calc_time = 0;
    static int64_t accumulated_us = 0;
    static bool ready_for_new_shot = true; // Latch
    
    int64_t now = esp_timer_get_time();

    // Case 1: Hard Reset (Pump Off)
    if (!coffee_data.shot_active) {
        accumulated_us = 0;
        coffee_data.shot_time_seconds = 0;
        last_calc_time = 0; 
        ready_for_new_shot = true; 
    } 
    // Case 2: Pump Running
    else {
        if (last_calc_time == 0) last_calc_time = now;

        if (coffee_data.pressure >= 1.0f) {
            // RISING EDGE: Start new timer
            if (ready_for_new_shot) {
                accumulated_us = 0;
                ready_for_new_shot = false; 
            }
            // Accumulate
            accumulated_us += (now - last_calc_time);
        } 
        else {
            // FALLING EDGE: Pause timer (Pre-infusion dwell)
            ready_for_new_shot = true;
        }
        
        coffee_data.shot_time_seconds = accumulated_us / 1000000;
        last_calc_time = now;
    }
}

// =============================================================================
// UI CREATION FUNCTIONS
// =============================================================================

/**
 * @brief Sets up the main screen background.
 * @details Loads the `background` image asset and tiles it to cover the screen.
 */
static void create_radial_background_image(void) {
    lv_obj_set_style_bg_img_src(main_screen, &background, 0);
    lv_obj_set_style_bg_img_opa(main_screen, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_img_tiled(main_screen, true, 0);
    lv_obj_set_style_bg_color(main_screen, lv_color_hex(0xF5F5F5), 0);
    lv_obj_set_style_bg_opa(main_screen, LV_OPA_COVER, 0);
}

/**
 * @brief Creates the static header elements (Title, Subtitle).
 * @details Positions are hardcoded for the 1.85" round display bezel.
 */
static void create_title_section(void) {
    // Main title (Adjusted Y for round bezel)
    lv_obj_t *title = lv_label_create(main_screen);
    lv_label_set_text(title, "BREVILLE PROFUSION");
    lv_obj_set_style_text_color(title, COLOR_DARK_GREY, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 55);
    
    // Subtitle
    lv_obj_t *subtitle = lv_label_create(main_screen);
    lv_label_set_text(subtitle, "Variable Pressure Espresso");
    lv_obj_set_style_text_color(subtitle, COLOR_LIGHT_GREY, 0);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_16, 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 78);
    
    // Connection Dot
    connection_status_label = lv_label_create(main_screen);
    lv_label_set_text(connection_status_label, "●");
    lv_obj_set_style_text_color(connection_status_label, COLOR_ERROR_RED, 0);
    lv_obj_set_style_text_font(connection_status_label, &lv_font_montserrat_16, 0);
    lv_obj_align(connection_status_label, LV_ALIGN_TOP_RIGHT, -10, 20);
}

/**
 * @brief Creates the main pressure readout and progress bar.
 */
static void create_pressure_section(void) {
    // Big Number Display
    pressure_label = lv_label_create(main_screen);
    lv_label_set_text(pressure_label, "0.0 bar");
    lv_obj_set_style_text_color(pressure_label, COLOR_PRESSURE, 0);
    lv_obj_set_style_text_font(pressure_label, &lv_font_montserrat_46, 0);
    lv_obj_align(pressure_label, LV_ALIGN_CENTER, 0, 10);
    
    // Header Label
    lv_obj_t *pressure_title = lv_label_create(main_screen);
    lv_label_set_text(pressure_title, "PRESSURE");
    lv_obj_set_style_text_color(pressure_title, COLOR_DARK_GREY, 0);
    lv_obj_set_style_text_font(pressure_title, &lv_font_montserrat_16, 0);
    lv_obj_align(pressure_title, LV_ALIGN_CENTER, 0, -33);
    
    // Progress Bar
    progress_bar = lv_bar_create(main_screen);
    lv_obj_set_size(progress_bar, 220, 20);
    lv_obj_align(progress_bar, LV_ALIGN_CENTER, 0, 48);
    lv_obj_set_style_bg_color(progress_bar, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_bg_color(progress_bar, COLOR_MEDIUM_GREY, LV_PART_INDICATOR);
    lv_obj_set_style_radius(progress_bar, 10, 0);
    lv_bar_set_value(progress_bar, 0, LV_ANIM_OFF);
}

/**
 * @brief Creates the temperature readout labels (GH and TB).
 */
static void create_temperature_section(void) {
    // Group Head
    temp_gh_label = lv_label_create(main_screen);
    lv_label_set_text(temp_gh_label, "GH: No Sensor");
    lv_obj_set_style_text_color(temp_gh_label, COLOR_BLACK, 0);
    lv_obj_set_style_text_font(temp_gh_label, &lv_font_montserrat_18, 0);
    lv_obj_align(temp_gh_label, LV_ALIGN_CENTER, -80, -72);
    
    // Thermoblock
    temp_tb_label = lv_label_create(main_screen);
    lv_label_set_text(temp_tb_label, "TB: No Sensor");
    lv_obj_set_style_text_color(temp_tb_label, COLOR_BLACK, 0);
    lv_obj_set_style_text_font(temp_tb_label, &lv_font_montserrat_18, 0);
    lv_obj_align(temp_tb_label, LV_ALIGN_CENTER, 80, -72);

    // Target Temp (for manual/auto feedback)
    target_temp_label = lv_label_create(main_screen);
    lv_label_set_text(target_temp_label, "Set: --");
    lv_obj_set_style_text_color(target_temp_label, COLOR_BLACK, 0);
    lv_obj_set_style_text_font(target_temp_label, &lv_font_montserrat_14, 0);
    lv_obj_align(target_temp_label, LV_ALIGN_CENTER, -80, -55); 
}

/**
 * @brief Creates the status indicators (Timer, Mode, Profile Name).
 */
static void create_status_section(void) {
    // Timer
    timer_label = lv_label_create(main_screen);
    lv_label_set_text(timer_label, "Timer: 00:00");
    lv_obj_set_style_text_color(timer_label, COLOR_BLACK, 0);
    lv_obj_set_style_text_font(timer_label, &lv_font_montserrat_20, 0);
    lv_obj_align(timer_label, LV_ALIGN_CENTER, -80, 75);
    
    // Mode
    mode_label = lv_label_create(main_screen);
    lv_label_set_text(mode_label, "Mode: AUTO");
    lv_obj_set_style_text_color(mode_label, COLOR_BLACK, 0);
    lv_obj_set_style_text_font(mode_label, &lv_font_montserrat_20, 0);
    lv_obj_align(mode_label, LV_ALIGN_CENTER, 80, 75);
    
    // Profile Name
    profile_label = lv_label_create(main_screen);
    lv_label_set_text(profile_label, "Standard");
    lv_obj_set_style_text_color(profile_label, COLOR_LIGHT_GREY, 0);
    lv_obj_set_style_text_font(profile_label, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_align(profile_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(profile_label, 140);
    lv_label_set_long_mode(profile_label, LV_LABEL_LONG_DOT);
    lv_obj_align(profile_label, LV_ALIGN_CENTER, 0, 95);
    
    // Main Status Text (Ready/Brewing)
    status_label = lv_label_create(main_screen);
    lv_label_set_text(status_label, "Disconnected");
    lv_obj_set_style_text_color(status_label, COLOR_ERROR_RED, 0);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_24, 0);
    lv_obj_align(status_label, LV_ALIGN_CENTER, 0, 135);
}

// =============================================================================
// DISPLAY UPDATE ROUTINES
// =============================================================================

/**
 * @brief Updates the pressure label string.
 * @details Throttles updates to 2Hz to prevent visual flickering of the digits.
 */
static void update_pressure_display(void) {
    // Throttle update to 2Hz
    static uint32_t last_pressure_update_time = 0;
    uint32_t now = esp_log_timestamp(); 
    
    if (now - last_pressure_update_time < 500) return; 
    
    if (coffee_data.pressure != coffee_data.last_pressure) {
        const char* pressure_str = get_pressure_string(coffee_data.pressure);
        lv_label_set_text(pressure_label, pressure_str);
        coffee_data.last_pressure = coffee_data.pressure;
        last_pressure_update_time = now; 
    }
    lv_obj_set_style_text_color(pressure_label, COLOR_PRESSURE, 0);
}

/**
 * @brief Updates temperature labels (GH, TB) and the "Set:" target label.
 * @details Only triggers a text update if the value has physically changed.
 */
static void update_temperature_display(void) {
    // 1. Group Head
    if (coffee_data.temp_grouphead != coffee_data.last_temp_gh) {
        format_temperature_gh(coffee_data.temp_grouphead);
        lv_label_set_text(temp_gh_label, temp_display_buffer_gh);
        
        if (coffee_data.temp_grouphead > -50.0f) {
            lv_obj_set_style_text_color(temp_gh_label, COLOR_BLACK, 0);
        } else {
            lv_obj_set_style_text_color(temp_gh_label, COLOR_LIGHT_GREY, 0);
        }
        coffee_data.last_temp_gh = coffee_data.temp_grouphead;
    }
    
    // 2. Thermoblock
    if (coffee_data.temp_thermoblock != coffee_data.last_temp_tb) {
        format_temperature_tb(coffee_data.temp_thermoblock);
        lv_label_set_text(temp_tb_label, temp_display_buffer_tb);
        
        if (coffee_data.temp_thermoblock > -50.0f) {
            lv_obj_set_style_text_color(temp_tb_label, COLOR_BLACK, 0);
        } else {
            lv_obj_set_style_text_color(temp_tb_label, COLOR_LIGHT_GREY, 0);
        }
        coffee_data.last_temp_tb = coffee_data.temp_thermoblock;
    }

    // 3. Target Temp Label (Always visible unless in Bypass)
    const ArduinoMachineState* s = get_arduino_state();
    if (s && coffee_data.operating_mode != MODE_BYPASS) {
        char buf[16];
        snprintf(buf, sizeof(buf), "Set: %.0f\xB0C", s->targetTemp);
        lv_label_set_text(target_temp_label, buf);
        lv_obj_clear_flag(target_temp_label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(target_temp_label, LV_OBJ_FLAG_HIDDEN);
    }
}

/**
 * @brief Updates timer, mode status, and bottom status text.
 * @details Handles logic for "Ready", "Brewing", and "Disconnected" states.
 */
static void update_status_display(void) {
    // Timer & Mode
    format_timer(coffee_data.shot_time_seconds);
    lv_label_set_text(timer_label, timer_display_buffer);
    
    const char* mode_text = "Mode: ???";
    switch (coffee_data.operating_mode) {
        case MODE_AUTO: mode_text = "Mode: AUTO"; break;
        case MODE_MANUAL: mode_text = "Mode: MANUAL"; break;
        case MODE_BYPASS: mode_text = "Mode: BYPASS"; break;
    }
    lv_label_set_text(mode_label, mode_text);

    // Profile Name / Manual Target Display
    if (coffee_data.operating_mode == MODE_AUTO) {
        lv_obj_clear_flag(profile_label, LV_OBJ_FLAG_HIDDEN);
        const char* name = get_profile_name(coffee_data.selected_profile);
        if (strcmp(lv_label_get_text(profile_label), name) != 0) {
            lv_label_set_text(profile_label, name);
        }
    } 
    else if (coffee_data.operating_mode == MODE_MANUAL) {
        lv_obj_clear_flag(profile_label, LV_OBJ_FLAG_HIDDEN);
        const ArduinoMachineState* s = get_arduino_state();
        float target = s ? s->targetPressure : 0.0f;
        char buf[32];
        snprintf(buf, sizeof(buf), "Target: %.1f bar", target);
        if (strcmp(lv_label_get_text(profile_label), buf) != 0) {
            lv_label_set_text(profile_label, buf);
        }
    } 
    else {
        lv_obj_add_flag(profile_label, LV_OBJ_FLAG_HIDDEN);
    }

    // Status Text & Color Logic
    char state_text[32];
    lv_color_t state_color;
    
    if (!coffee_data.arduino_connected) {
        strncpy(state_text, "Disconnected", sizeof(state_text));
        state_color = COLOR_ERROR_RED; 
        lv_obj_set_style_text_color(connection_status_label, COLOR_ERROR_RED, 0);
    } 
    else if (coffee_data.shot_active) {
        if (coffee_data.pressure >= 1.0f) {
            strncpy(state_text, "Brewing", sizeof(state_text));
            state_color = COLOR_BLACK;
        } else {
            // Low Pressure + Active = Pre-Infusion (Auto) or Ready (Manual)
            if (coffee_data.operating_mode == MODE_AUTO) {
                strncpy(state_text, "Pre-Infusion", sizeof(state_text));
                state_color = COLOR_DARK_GREY;
            } else {
                get_thermal_status_text(state_text, sizeof(state_text), &state_color);
            }
        }
        lv_obj_set_style_text_color(connection_status_label, COLOR_SUCCESS_GREEN, 0);
    } 
    else {
        // Idle State: Show Thermal Status
        get_thermal_status_text(state_text, sizeof(state_text), &state_color);
        lv_obj_set_style_text_color(connection_status_label, COLOR_SUCCESS_GREEN, 0);
    }
    
    lv_label_set_text(status_label, state_text);
    lv_obj_set_style_text_color(status_label, state_color, 0);
    
    // Progress Bar Animation
    if (coffee_data.shot_active && coffee_data.shot_time_seconds > 0) {
        int progress = (coffee_data.shot_time_seconds * 100) / 30; // Assume 30s max for scale
        progress = (progress > 100) ? 100 : progress;
        lv_bar_set_value(progress_bar, progress, LV_ANIM_OFF);
    } else {
        lv_bar_set_value(progress_bar, 0, LV_ANIM_OFF);
    }
}

// =============================================================================
// PUBLIC API IMPLEMENTATION
// =============================================================================

void Lvgl_Coffee_UI_Init(void) {
    ESP_LOGI(TAG, "Coffee UI initialization starting (V46)");
    queue_debug_message(2, "ESP32 UI starting");
    
    main_screen = lv_obj_create(NULL);
    create_radial_background_image();
    lv_scr_load(main_screen);
    
    create_title_section();
    create_pressure_section();
    create_temperature_section();
    create_status_section();
    
    queue_debug_message(2, "UI initialized");

    // Force initial status update
    coffee_data.arduino_connected = is_arduino_connected();
    if (coffee_data.arduino_connected) {
        update_status_display();
    }
    ESP_LOGI(TAG, "Coffee UI initialization complete");
}

void Lvgl_Coffee_UI_Update(void) {
    update_coffee_data_from_arduino();
    update_pressure_display();
    update_temperature_display();
    update_status_display();
}

void Lvgl_Coffee_UI_Loop(void) {
    static int update_counter = 0;
    // Limit UI refresh to 10Hz (every 10 calls if main loop is 100Hz)
    if (++update_counter >= 10) {
        Lvgl_Coffee_UI_Update();
        update_counter = 0;
    }
}