// Single file for all user-tunable constants — change values here, not in main.c.
#ifndef SETTINGS_H
#define SETTINGS_H

// =============================================================================
// USER PREFERENCES
// =============================================================================
#define LCD_BACKLIGHT_LEVEL     70      // Screen Brightness (0-100%)
#define LOG_TO_SD_ENABLED       1       // 1 = Enable CSV logging, 0 = Disable

// =============================================================================
// PIN DEFINITIONS (HARDWARE CONFIG)
// =============================================================================
// ESP32-S3 I2C Pins
#define I2C_MASTER_SDA_IO       11
#define I2C_MASTER_SCL_IO       10

// =============================================================================
// I2C COMMUNICATION SETTINGS
// =============================================================================
#define ARDUINO_I2C_ADDRESS     0x08    // Slave Address of the Arduino Uno
#define I2C_MASTER_FREQ_HZ      50000   // 50kHz (Slow/Robust for internal wiring)
#define I2C_MASTER_TIMEOUT_MS   1000

// =============================================================================
// ENERGY SAVING & AUTOMATION
// =============================================================================
#define TB_SLEEP_THRESHOLD_C    25.0f   // Temp below which ESP32 enters "Sleep Mode"
#define TB_WAKE_THRESHOLD_C     26.0f   // Hysteresis buffer to wake up
#define TB_OFF_THRESHOLD_C      40.0f   // Safety: Auto-Start only fires if temp is below this

// Polling intervals (ms)
#define SLEEP_I2C_POLL_MS       250     // Poll slower when machine is "Off" (4Hz)
#define ACTIVE_I2C_POLL_MS      100     // Poll fast when machine is "On" (10Hz)

// =============================================================================
// LOGGING CONFIGURATION
// =============================================================================
#define LOG_FILE_PATH           "/sdcard/brew_log.csv"
#define LOG_BACKUP_PATH         "/sdcard/brew_log.bak"
#define MAX_LOG_SIZE_BYTES      (5 * 1024 * 1024) // 5MB limit before rotation

#endif // SETTINGS_H