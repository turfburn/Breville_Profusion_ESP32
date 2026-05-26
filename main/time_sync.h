/**
 * @file time_sync.h
 * @brief WiFi and NTP time synchronization
 */

#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdbool.h>
#include <time.h>

/**
 * @brief Main time initialization function
 * @details Attempts to set time via: RTC → WiFi+NTP → Manual
 * Call this in app_main() after drivers_init()
 */
void time_init(void);

/**
 * @brief Set system time manually
 * @param year Full year (e.g., 2025)
 * @param month Month (1-12)
 * @param day Day (1-31)
 * @param hour Hour (0-23)
 * @param minute Minute (0-59)
 * @param second Second (0-59)
 */
void set_time_manually(int year, int month, int day, int hour, int minute, int second);

/**
 * @brief Read time from hardware RTC
 * @return true if RTC has valid time
 */
bool read_time_from_rtc(void);

/**
 * @brief Print current system time for debugging
 */
void print_current_time(void);

/**
 * @brief Initialize WiFi in station mode
 * @return true if connected
 */
bool wifi_init_sta(void);

/**
 * @brief Sync time via SNTP
 */
void sync_time_from_ntp(void);

#endif // TIME_SYNC_H
