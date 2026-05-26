/**
 * @file profile_manager.c
 * @brief Profile and schedule management implementation
 */

#include "profile_manager.h"
#include "esp_log.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <inttypes.h>

static const char *TAG = "PROFILE_MGR";

// Global storage
brew_profile_t g_profiles[MAX_PROFILES];
schedule_t g_schedule;
profile_constraints_t g_constraints;
int g_num_valid_profiles = 0;

// =============================================================================
// INITIALIZATION
// =============================================================================

bool profile_manager_init(void) {
    ESP_LOGI(TAG, "Initializing profile manager");
    
    g_constraints.max_points_per_profile = DEFAULT_MAX_POINTS;
    g_constraints.max_profile_duration_s = DEFAULT_MAX_DURATION_S;
    g_constraints.min_time_between_points_s = DEFAULT_MIN_TIME_BETWEEN_POINTS_S;
    g_constraints.min_pressure_bar = DEFAULT_MIN_PRESSURE_BAR;
    g_constraints.max_pressure_bar = DEFAULT_MAX_PRESSURE_BAR;
    g_constraints.safe_max_pressure_bar = DEFAULT_SAFE_MAX_PRESSURE_BAR;
    g_constraints.min_temp_c = DEFAULT_MIN_TEMP_C;
    g_constraints.max_temp_c = DEFAULT_MAX_TEMP_C;
    g_constraints.safe_max_temp_c = DEFAULT_SAFE_MAX_TEMP_C;
    g_constraints.preinfusion_min_pressure_bar = 1.0f;
    g_constraints.preinfusion_max_pressure_bar = 4.0f;
    g_constraints.preinfusion_min_duration_ms = 1000;
    g_constraints.preinfusion_max_duration_ms = 10000;
    
    bool profiles_loaded = load_profiles_from_sd();
    bool schedule_loaded = load_schedule_from_sd();
    
    if (!profiles_loaded) {
        ESP_LOGW(TAG, "Using default profiles");
        load_default_profiles();
    }
    
    if (!schedule_loaded) {
        ESP_LOGW(TAG, "Using default schedule");
        load_default_schedule();
    }
    
    ESP_LOGI(TAG, "Profile manager initialized - %d valid profiles", g_num_valid_profiles);
    return true;
}

static char* read_file_to_string(const char *filepath) {
    FILE *f = fopen(filepath, "r");
    if (!f) { return NULL; }
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fsize <= 0 || fsize > 50000) { fclose(f); return NULL; }
    char *buffer = malloc(fsize + 1);
    if (!buffer) { fclose(f); return NULL; }
    size_t read_size = fread(buffer, 1, fsize, f);
    fclose(f);
    if (read_size != fsize) { free(buffer); return NULL; }
    buffer[fsize] = '\0';
    return buffer;
}

bool validate_profile(const brew_profile_t *profile) {
    if (!profile) return false;
    if (profile->duration_seconds > g_constraints.max_profile_duration_s) return false;
    if (profile->target_pressure_bar > g_constraints.max_pressure_bar) return false;
    return true;
}

bool validate_schedule(const schedule_t *schedule) {
    if (!schedule) return false;
    return true;
}

static bool parse_profile_point_array(cJSON *array, profile_point_t *points, int *num_points, int max_points) {
    if (!cJSON_IsArray(array)) return false;
    int count = cJSON_GetArraySize(array);
    if (count > max_points) return false;
    
    for (int i = 0; i < count; i++) {
        cJSON *point = cJSON_GetArrayItem(array, i);
        cJSON *time = cJSON_GetObjectItem(point, "time_s");
        cJSON *value_pressure = cJSON_GetObjectItem(point, "pressure_bar");
        cJSON *value_temp = cJSON_GetObjectItem(point, "temperature_c");
        
        if (!cJSON_IsNumber(time)) return false;
        points[i].time_s = (float)time->valuedouble;
        
        if (cJSON_IsNumber(value_pressure)) points[i].value = (float)value_pressure->valuedouble;
        else if (cJSON_IsNumber(value_temp)) points[i].value = (float)value_temp->valuedouble;
        else return false;
    }
    *num_points = count;
    return true;
}

bool load_profiles_from_sd(void) {
    char *json_str = read_file_to_string(PROFILES_PATH);
    if (!json_str) return false;
    cJSON *root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) return false;
    
    cJSON *profiles_array = cJSON_GetObjectItem(root, "profiles");
    if (!cJSON_IsArray(profiles_array)) { cJSON_Delete(root); return false; }
    
    int count = cJSON_GetArraySize(profiles_array);
    g_num_valid_profiles = 0;
    
    for (int i = 0; i < count && i < MAX_PROFILES; i++) {
        cJSON *prof = cJSON_GetArrayItem(profiles_array, i);
        brew_profile_t *profile = &g_profiles[i];
        memset(profile, 0, sizeof(brew_profile_t));
        
        cJSON *id = cJSON_GetObjectItem(prof, "id");
        cJSON *name = cJSON_GetObjectItem(prof, "name");
        cJSON *desc = cJSON_GetObjectItem(prof, "description");
        
        // === NEW PREHEAT PARSING ===
        cJSON *preheat = cJSON_GetObjectItem(prof, "preheat_temperature_c");
        if (cJSON_IsNumber(preheat)) profile->preheat_temperature_c = (float)preheat->valuedouble;
        else profile->preheat_temperature_c = 93.0f; // Default
        
        cJSON *duration = cJSON_GetObjectItem(prof, "duration_seconds");
        cJSON *temp = cJSON_GetObjectItem(prof, "target_temperature_c");
        cJSON *pressure = cJSON_GetObjectItem(prof, "target_pressure_bar");
        
        if (cJSON_IsNumber(id)) profile->id = id->valueint;
        if (cJSON_IsString(name)) strncpy(profile->name, name->valuestring, PROFILE_NAME_MAX_LEN - 1);
        if (cJSON_IsString(desc)) strncpy(profile->description, desc->valuestring, 63);
        if (cJSON_IsNumber(duration)) profile->duration_seconds = duration->valueint;
        if (cJSON_IsNumber(temp)) profile->target_temperature_c = (float)temp->valuedouble;
        if (cJSON_IsNumber(pressure)) profile->target_pressure_bar = (float)pressure->valuedouble;
        
        cJSON *preinf = cJSON_GetObjectItem(prof, "preinfusion");
        if (cJSON_IsObject(preinf)) {
            profile->preinfusion.enabled = cJSON_IsTrue(cJSON_GetObjectItem(preinf, "enabled"));
            cJSON *pp = cJSON_GetObjectItem(preinf, "pressure_bar");
            if(cJSON_IsNumber(pp)) profile->preinfusion.pressure_bar = pp->valuedouble;
            cJSON *pd = cJSON_GetObjectItem(preinf, "duration_ms");
            if(cJSON_IsNumber(pd)) profile->preinfusion.duration_ms = pd->valueint;
        }
        
        cJSON *pressure_profile = cJSON_GetObjectItem(prof, "pressure_profile");
        if (cJSON_IsObject(pressure_profile)) {
            parse_profile_point_array(cJSON_GetObjectItem(pressure_profile, "points"), 
                                    profile->pressure_points, &profile->num_pressure_points, MAX_POINTS);
        }
        
        cJSON *temp_profile = cJSON_GetObjectItem(prof, "temperature_profile");
        if (cJSON_IsObject(temp_profile)) {
            parse_profile_point_array(cJSON_GetObjectItem(temp_profile, "points"), 
                                    profile->temp_points, &profile->num_temp_points, MAX_POINTS);
        }
        
        profile->is_valid = true;
        g_num_valid_profiles++;
    }
    
    cJSON_Delete(root);
    return (g_num_valid_profiles > 0);
}

bool load_schedule_from_sd(void) {
    char *json_str = read_file_to_string(SCHEDULE_PATH);
    if (!json_str) return false;
    cJSON *root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) return false;
    
    cJSON *schedule_array = cJSON_GetObjectItem(root, "schedule");
    if (!cJSON_IsArray(schedule_array)) { cJSON_Delete(root); return false; }
    
    memset(&g_schedule, 0, sizeof(schedule_t));
    int count = cJSON_GetArraySize(schedule_array);
    
    ESP_LOGI(TAG, "Loading Schedule...");

    for (int i = 0; i < count && i < SCHEDULE_DAYS; i++) {
        cJSON *day = cJSON_GetArrayItem(schedule_array, i);
        cJSON *day_index = cJSON_GetObjectItem(day, "day_index");
        cJSON *enabled = cJSON_GetObjectItem(day, "enabled");
        cJSON *start_time = cJSON_GetObjectItem(day, "start_time");
        
        if (cJSON_IsNumber(day_index)) {
            int idx = day_index->valueint;
            if (idx >= 0 && idx < SCHEDULE_DAYS) {
                g_schedule.days[idx].day_index = idx;
                g_schedule.days[idx].enabled = cJSON_IsTrue(enabled);
                
                // FIX: Use standard int parsing to avoid %hhu issues
                if (cJSON_IsString(start_time)) {
                    int h, m;
                    if (sscanf(start_time->valuestring, "%d:%d", &h, &m) == 2) {
                        g_schedule.days[idx].hour = (uint8_t)h;
                        g_schedule.days[idx].minute = (uint8_t)m;
                        
                        // DEBUG LOG: Verify what we loaded
                        if (g_schedule.days[idx].enabled) {
                            ESP_LOGI(TAG, "  Day %d: %02d:%02d (Enabled)", idx, h, m);
                        }
                    } else {
                        ESP_LOGW(TAG, "  Day %d: Invalid Time Format '%s'", idx, start_time->valuestring);
                    }
                }
            }
        }
    }
    
    cJSON_Delete(root);
    g_schedule.is_valid = true;
    return true;
}

bool save_profiles_to_sd(void) { return false; }
bool save_schedule_to_sd(void) { return false; }
bool backup_profiles(void) { return false; }
bool restore_profiles_from_backup(void) { return false; }
bool backup_schedule(void) { return false; }
bool restore_schedule_from_backup(void) { return false; }

static float interpolate(float t, float t0, float t1, float v0, float v1) {
    if (t <= t0) return v0;
    if (t >= t1) return v1;
    return v0 + (t - t0) / (t1 - t0) * (v1 - v0);
}

float get_target_pressure(int profile_id, float elapsed_seconds) {
    if (profile_id < 0 || profile_id >= MAX_PROFILES) return 0.0f;
    brew_profile_t *p = &g_profiles[profile_id];
    if (!p->is_valid) return 0.0f;
    if (p->num_pressure_points == 0) return p->target_pressure_bar;
    
    if (elapsed_seconds <= p->pressure_points[0].time_s) return p->pressure_points[0].value;
    if (elapsed_seconds >= p->pressure_points[p->num_pressure_points-1].time_s) 
        return p->pressure_points[p->num_pressure_points-1].value;
        
    for (int i = 0; i < p->num_pressure_points - 1; i++) {
        if (elapsed_seconds >= p->pressure_points[i].time_s && elapsed_seconds <= p->pressure_points[i+1].time_s) {
            return interpolate(elapsed_seconds, p->pressure_points[i].time_s, p->pressure_points[i+1].time_s,
                             p->pressure_points[i].value, p->pressure_points[i+1].value);
        }
    }
    return p->target_pressure_bar;
}

float get_target_temperature(int profile_id, float elapsed_seconds) {
    if (profile_id < 0 || profile_id >= MAX_PROFILES) return 0.0f;
    brew_profile_t *p = &g_profiles[profile_id];
    if (!p->is_valid) return 0.0f;
    if (p->num_temp_points == 0) return p->target_temperature_c;
    
    if (elapsed_seconds <= p->temp_points[0].time_s) return p->temp_points[0].value;
    if (elapsed_seconds >= p->temp_points[p->num_temp_points-1].time_s) 
        return p->temp_points[p->num_temp_points-1].value;
        
    for (int i = 0; i < p->num_temp_points - 1; i++) {
        if (elapsed_seconds >= p->temp_points[i].time_s && elapsed_seconds <= p->temp_points[i+1].time_s) {
            return interpolate(elapsed_seconds, p->temp_points[i].time_s, p->temp_points[i+1].time_s,
                             p->temp_points[i].value, p->temp_points[i+1].value);
        }
    }
    return p->target_temperature_c;
}

const brew_profile_t* get_profile(int profile_id) {
    if (profile_id < 0 || profile_id >= MAX_PROFILES) return NULL;
    return &g_profiles[profile_id];
}

const schedule_entry_t* get_schedule_for_day(int day_index) {
    if (day_index < 0 || day_index >= SCHEDULE_DAYS) return NULL;
    return &g_schedule.days[day_index];
}

void load_default_profiles(void) {
    memset(g_profiles, 0, sizeof(g_profiles));
    
    // --- PROFILE 0: LIGHT ROAST (Hot / Fast) ---
    g_profiles[0].is_valid = true; 
    g_profiles[0].id = 0;
    strcpy(g_profiles[0].name, "Deflt Light");
    strcpy(g_profiles[0].description, "High temp, max pressure");
    g_profiles[0].preheat_temperature_c = 95.0f;
    g_profiles[0].target_temperature_c = 95.0f;
    g_profiles[0].target_pressure_bar = 9.0f;
    g_profiles[0].duration_seconds = 30;
    
    // --- PROFILE 1: MEDIUM ROAST (Standard) ---
    g_profiles[1].is_valid = true;
    g_profiles[1].id = 1;
    strcpy(g_profiles[1].name, "Deflt Medium");
    strcpy(g_profiles[1].description, "Standard 9 bar shot");
    g_profiles[1].preheat_temperature_c = 93.0f;
    g_profiles[1].target_temperature_c = 93.0f;
    g_profiles[1].target_pressure_bar = 9.0f;
    g_profiles[1].duration_seconds = 30;

    // --- PROFILE 2: DARK ROAST (Cool / Gentle) ---
    g_profiles[2].is_valid = true;
    g_profiles[2].id = 2;
    strcpy(g_profiles[2].name, "Deflt Dark");
    strcpy(g_profiles[2].description, "Lower temp and pressure");
    g_profiles[2].preheat_temperature_c = 88.0f; // Cool start
    g_profiles[2].target_temperature_c = 88.0f;
    g_profiles[2].target_pressure_bar = 8.0f;    // Gentle pressure
    g_profiles[2].duration_seconds = 25;         // Shorter shot

    g_num_valid_profiles = 3;
    ESP_LOGI(TAG, "Loaded 3 Default Profiles (SD Fallback)");
}

void load_default_schedule(void) {
    memset(&g_schedule, 0, sizeof(schedule_t));
    g_schedule.is_valid = true;
}

const char* get_profile_name(uint8_t index) {
    const brew_profile_t* p = get_profile(index);
    if (p && p->is_valid) {
        return p->name;
    }
    return "Unknown";
}