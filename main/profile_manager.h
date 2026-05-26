/**
 * @file profile_manager.h
 * @brief Profile and schedule management for ESP32 coffee machine
 */

#ifndef PROFILE_MANAGER_H
#define PROFILE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// Configuration
#define MAX_PROFILES 3
#define MAX_POINTS 20
#define PROFILE_NAME_MAX_LEN 32
#define SCHEDULE_DAYS 7

// File paths on SD card
#define PROFILES_PATH "/sdcard/profiles.json"
#define SCHEDULE_PATH "/sdcard/schedule.json"
#define PROFILES_BACKUP_PATH "/sdcard/profiles.backup.json"
#define SCHEDULE_BACKUP_PATH "/sdcard/schedule.backup.json"

#define DEFAULT_MAX_POINTS 20
#define DEFAULT_MAX_DURATION_S 120
#define DEFAULT_MIN_TIME_BETWEEN_POINTS_S 0.5f
#define DEFAULT_MIN_PRESSURE_BAR 0.0f
#define DEFAULT_MAX_PRESSURE_BAR 12.0f
#define DEFAULT_SAFE_MAX_PRESSURE_BAR 10.5f
#define DEFAULT_MIN_TEMP_C 80.0f
#define DEFAULT_MAX_TEMP_C 100.0f
#define DEFAULT_SAFE_MAX_TEMP_C 96.0f

// One node in a pressure or temperature curve; get_target_pressure/temperature
// linearly interpolates between adjacent nodes at runtime.
typedef struct {
    float time_s;
    float value;
} profile_point_t;

typedef struct {
    bool enabled;
    float pressure_bar;
    uint32_t duration_ms;
} preinfusion_config_t;

// Complete description of a brew: target values, pre-infusion config, and up to
// MAX_POINTS nodes each for the pressure and temperature curves over time.
// Persisted as JSON on the SD card; loaded at boot by profile_manager_init().
typedef struct {
    int id;
    char name[PROFILE_NAME_MAX_LEN];
    char description[64];

    float preheat_temperature_c;
    
    uint32_t duration_seconds;
    float target_temperature_c;
    float target_pressure_bar;
    preinfusion_config_t preinfusion;
    profile_point_t pressure_points[MAX_POINTS];
    int num_pressure_points;
    profile_point_t temp_points[MAX_POINTS];
    int num_temp_points;
    bool is_valid;
} brew_profile_t;

// Per-day auto-wake configuration.  When enabled, check_schedule_wake() in main.c
// pulses the Arduino power button within a 15-minute window of hour:minute.
typedef struct {
    char day_name[16];
    int day_index;   // 0=Sunday … 6=Saturday
    bool enabled;
    uint8_t hour;    // 0–23
    uint8_t minute;  // 0–59
} schedule_entry_t;

typedef struct {
    schedule_entry_t days[SCHEDULE_DAYS];
    bool is_valid;
} schedule_t;

typedef struct {
    int max_points_per_profile;
    int max_profile_duration_s;
    float min_time_between_points_s;
    float min_pressure_bar;
    float max_pressure_bar;
    float safe_max_pressure_bar;
    float min_temp_c;
    float max_temp_c;
    float safe_max_temp_c;
    float preinfusion_min_pressure_bar;
    float preinfusion_max_pressure_bar;
    uint32_t preinfusion_min_duration_ms;
    uint32_t preinfusion_max_duration_ms;
} profile_constraints_t;

extern brew_profile_t g_profiles[MAX_PROFILES];
extern schedule_t g_schedule;
extern profile_constraints_t g_constraints;
extern int g_num_valid_profiles;

bool profile_manager_init(void);
bool load_profiles_from_sd(void);
bool load_schedule_from_sd(void);
bool save_profiles_to_sd(void);
bool save_schedule_to_sd(void);

bool backup_profiles(void);
bool restore_profiles_from_backup(void);
bool backup_schedule(void);
bool restore_schedule_from_backup(void);

float get_target_pressure(int profile_id, float elapsed_seconds);
float get_target_temperature(int profile_id, float elapsed_seconds);
const brew_profile_t* get_profile(int profile_id);
const schedule_entry_t* get_schedule_for_day(int day_index);

bool validate_profile(const brew_profile_t *profile);
bool validate_schedule(const schedule_t *schedule);

void load_default_profiles(void);
void load_default_schedule(void);

const char* get_profile_name(uint8_t index);

#endif // PROFILE_MANAGER_H