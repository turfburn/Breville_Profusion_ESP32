/**
 * @file time_sync.c
 * @brief Passive Time & Hardware RTC Manager
 * @version V2.0 - Production
 * @date 2025-11-29
 * @author [Your Name/Project Name]
 * * @details
 * This module manages the interface between the ESP32's system time (software)
 * and the external PCF85063 Real-Time Clock (hardware).
 * * **Key Responsibilities:**
 * 1. **Boot Restore:** On startup, reads the RTC to set the system time immediately,
 * ensuring valid timestamps before Wi-Fi connects.
 * 2. **Drift Correction:** When NTP updates occurs (via `Wireless.c`), this module
 * checks if the hardware RTC has drifted significantly (>5 mins) before writing
 * to it, preserving write cycles.
 * 3. **Format Conversion:** Handles conversion between `struct tm` and the RTC's
 * internal BCD format.
 * * @note
 * Network time (SNTP) initialization is handled by `Wireless.c`, which calls
 * `update_rtc_from_system()` as a callback.
 */

#include "esp_log.h"
#include <time.h>
#include <sys/time.h>
#include <stdlib.h> // For abs()
#include "PCF85063.h"

static const char *TAG = "TIME_SYNC";

// =============================================================================
// INTERNAL HELPERS: TIME FORMAT CONVERSION
// =============================================================================

/**
 * @brief Convert standard C `struct tm` to RTC driver `datetime_t`.
 * @param tm_time Pointer to source struct tm (from system).
 * @param dt_time Pointer to destination datetime_t (for RTC).
 */
static void tm_to_datetime(const struct tm *tm_time, datetime_t *dt_time) {
    dt_time->year = tm_time->tm_year + 1900;
    dt_time->month = tm_time->tm_mon + 1;
    dt_time->day = tm_time->tm_mday;
    dt_time->dotw = tm_time->tm_wday;
    dt_time->hour = tm_time->tm_hour;
    dt_time->minute = tm_time->tm_min;
    dt_time->second = tm_time->tm_sec;
}

/**
 * @brief Convert RTC driver `datetime_t` to standard C `struct tm`.
 * @param dt_time Pointer to source datetime_t (from RTC).
 * @param tm_time Pointer to destination struct tm (for system).
 */
static void datetime_to_tm(const datetime_t *dt_time, struct tm *tm_time) {
    tm_time->tm_year = dt_time->year - 1900;
    tm_time->tm_mon = dt_time->month - 1;
    tm_time->tm_mday = dt_time->day;
    tm_time->tm_wday = dt_time->dotw;
    tm_time->tm_hour = dt_time->hour;
    tm_time->tm_min = dt_time->minute;
    tm_time->tm_sec = dt_time->second;
    tm_time->tm_isdst = -1; // Let system determine DST
}

// =============================================================================
// RTC DRIVER WRAPPERS
// =============================================================================

void PCF85063_SetDateTime(struct tm *timeinfo) {
    datetime_t dt;
    tm_to_datetime(timeinfo, &dt);
    PCF85063_Set_All(dt);
}

bool PCF85063_GetDateTime(struct tm *timeinfo) {
    datetime_t dt;
    PCF85063_Read_Time(&dt);
    
    // Basic validation: Range 2024-2100
    if (dt.year < 2024 || dt.year > 2100 || 
        dt.month < 1 || dt.month > 12 ||
        dt.day < 1 || dt.day > 31) {
        return false;
    }
    
    datetime_to_tm(&dt, timeinfo);
    return true;
}

// =============================================================================
// PUBLIC API IMPLEMENTATION
// =============================================================================

/**
 * @brief Read time from hardware RTC and set system time.
 * @details Should be called early in boot process.
 * @return true if successful, false if RTC has invalid/reset time.
 */
bool read_time_from_rtc(void)
{
    struct tm timeinfo;
    if (!PCF85063_GetDateTime(&timeinfo)) {
        ESP_LOGW(TAG, "Failed to read valid time from RTC");
        return false;
    }
    
    // Check if time is valid (year should be reasonable)
    if (timeinfo.tm_year < (2024 - 1900)) {
        ESP_LOGW(TAG, "RTC has invalid time (Year: %d)", timeinfo.tm_year + 1900);
        return false;
    }
    
    // Set ESP32 System time from External RTC
    time_t t = mktime(&timeinfo);
    struct timeval now = { .tv_sec = t };
    settimeofday(&now, NULL);
    
    // Apply Chicago Timezone (Standard Project Default)
    setenv("TZ", "CST6CDT,M3.2.0,M11.1.0", 1);
    tzset();
    
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "System time restored from RTC: %s", strftime_buf);
    
    return true;
}

/**
 * @brief Helper to update RTC from System Time.
 * @details Called by `Wireless.c` after NTP sync event.
 * **Logic:**
 * 1. **Power Up:** Always Sync to force initial accuracy.
 * 2. **Periodic:** Check every 24h. Only write to RTC if drift > 5 mins.
 * This prevents unnecessary I2C traffic and EEPROM wear.
 */
void update_rtc_from_system(void) {
    static bool is_first_sync = true;
    static time_t last_check_time = 0;
    
    // Get Current System Time (NTP Accurate)
    time_t now_sys;
    struct tm timeinfo_sys;
    time(&now_sys);
    localtime_r(&now_sys, &timeinfo_sys);
    
    // Sanity check: Don't write garbage if NTP failed (Year 1970)
    if (timeinfo_sys.tm_year < (2024 - 1900)) {
        ESP_LOGW(TAG, "System time invalid, skipping RTC update");
        return;
    }

    // === SCENARIO 1: FIRST POWER UP ===
    if (is_first_sync) {
        ESP_LOGI(TAG, "Power Up: Forcing RTC Sync to ensure accuracy.");
        PCF85063_SetDateTime(&timeinfo_sys);
        is_first_sync = false;
        last_check_time = now_sys;
        return;
    }

    // === SCENARIO 2: PERIODIC CHECK (24 HOURS) ===
    // 86400 seconds = 24 hours
    if ((now_sys - last_check_time) > 86400) {
        ESP_LOGI(TAG, "24 Hours elapsed. Checking RTC drift...");
        
        // Read current RTC time
        struct tm timeinfo_rtc;
        if (PCF85063_GetDateTime(&timeinfo_rtc)) {
            time_t now_rtc = mktime(&timeinfo_rtc);
            
            // Calculate Drift (Absolute difference)
            double drift = difftime(now_sys, now_rtc); // System - RTC
            long abs_drift = labs((long)drift);
            
            ESP_LOGI(TAG, "Drift Check: System=%lld, RTC=%lld, Diff=%ld sec", 
                     (long long)now_sys, (long long)now_rtc, abs_drift);

            // 5 Minutes = 300 Seconds threshold
            if (abs_drift > 300) {
                ESP_LOGW(TAG, "Drift > 5 mins detected. Updating RTC.");
                PCF85063_SetDateTime(&timeinfo_sys);
            } else {
                ESP_LOGI(TAG, "Drift < 5 mins. RTC update skipped.");
            }
        }
        
        // Reset timer regardless of update result
        last_check_time = now_sys;
    }
}

/**
 * @brief Main time initialization function.
 * @details Call this in `app_main`. Attempts to restore time from hardware RTC.
 * Does NOT initialize Wi-Fi or SNTP (handled by Wireless module).
 */
void time_init(void)
{
    ESP_LOGI(TAG, "=== Schedule & RTC Initialization ===");
    
    // Try to restore time from RTC immediately
    if (read_time_from_rtc()) {
        ESP_LOGI(TAG, "RTC Restore Successful");
    } else {
        ESP_LOGW(TAG, "RTC Invalid/Empty. Waiting for WiFi NTP.");
    }
}