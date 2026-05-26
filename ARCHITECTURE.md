# Breville Profusion — Architecture & Function Reference

## System Overview

```
┌─────────────────────────────────────────────────┐
│  ESP32-S3 (this firmware)                       │
│                                                 │
│  ┌──────────┐  ┌──────────┐  ┌───────────────┐ │
│  │ LVGL UI  │  │ Profile  │  │  Scheduler /  │ │
│  │ (display)│  │ Manager  │  │  Power Mgmt   │ │
│  └────┬─────┘  └────┬─────┘  └──────┬────────┘ │
│       │             │               │           │
│  ┌────▼─────────────▼───────────────▼────────┐  │
│  │        i2c_queue (FreeRTOS queue)         │  │
│  │        i2c_communication_task             │  │
│  └────────────────────┬──────────────────────┘  │
│                       │ I2C (50 kHz, addr 0x08) │
└───────────────────────┼─────────────────────────┘
                        │
┌───────────────────────▼─────────────────────────┐
│  Arduino Uno slave                              │
│  • Pressure sensor                              │
│  • Group-head temperature (DS18B20)             │
│  • Thermoblock temperature (DS18B20)            │
│  • Pump PID                                     │
│  • Thermoblock heater PID                       │
└─────────────────────────────────────────────────┘
```

## Data Flow

```
Arduino state struct
  → i2c_communication_task polls at 10 Hz (active) / 4 Hz (sleep)
  → g_arduinoState global updated
  → get_arduino_state() exposes it read-only
  → Lvgl_Coffee_UI_Loop() reads it every 100 ms
  → LVGL labels updated on screen
```

---

## Module Reference

### `main/main.c` — Core Firmware

Entry point and orchestration.  Creates the I2C task, initialises all drivers,
and runs the main loop.

#### Key Structs

```c
// Byte-for-byte mirror of what the Arduino sends over I2C.
// __attribute__((packed)) is required — struct layout must match exactly.
typedef struct {
    float    pressure;         // Bar
    float    groupHeadTemp;    // °C
    float    thermoblockTemp;  // °C
    float    targetPressure;   // PID setpoint (bar)
    float    targetTemp;       // PID setpoint (°C)
    uint8_t  pumpPower;        // 0–100 %
    uint8_t  selectedProgram;  // Index into stored brew profiles
    uint8_t  operatingMode;    // 0=bypass, 1=manual, 2=auto
    uint8_t  stateFlags;       // Bit 0=shot active, Bit 1=manual, Bit 2=error
    uint8_t  heartbeat;        // Incremented each Arduino loop; used as comms watchdog
    uint8_t  padding;
} __attribute__((packed)) ArduinoMachineState;

typedef struct {
    bool          connected;
    unsigned long totalRequests;
    unsigned long successfulRequests;
    unsigned long lastSuccessTime;
    unsigned long consecutiveFailures;
    float         successRate;
} ArduinoCommStats;
```

#### Public Functions

| Function | Description |
|----------|-------------|
| `app_main()` | System entry point: initialises I2C queue, drivers, SD, profiles, WiFi, display, then runs the main loop |
| `get_arduino_state()` | Returns a read-only pointer to the current `ArduinoMachineState` |
| `get_arduino_stats()` | Returns a read-only pointer to `ArduinoCommStats` |
| `is_arduino_connected()` | True if fewer than 5 consecutive I2C failures |
| `queue_arduino_command(cmd, data, len)` | Enqueues a command byte + payload to be sent to the Arduino; safe to call from any task |
| `queue_debug_message(type, msg)` | Enqueues a telemetry string to be sent to the Arduino over I2C |
| `set_arduino_pid_targets(pressure, temp)` | Sends updated PID setpoints to the Arduino |
| `log_data_to_sd()` | Appends one CSV row to `/sdcard/brew_log.csv`; rotates to `.bak` at 5 MB |
| `process_active_shot()` | State machine for brew execution: interpolates profile curve and sends setpoints |
| `trigger_arduino_power_button()` | Sends a 0x20 command to simulate a power-button press on the Arduino |

---

### `main/settings.h` — User Configuration

Single location for all tuneable constants.  Change values here rather than
hunting through source files.

| Constant | Default | Purpose |
|----------|---------|---------|
| `LCD_BACKLIGHT_LEVEL` | 70 | Screen brightness 0–100 % |
| `LOG_TO_SD_ENABLED` | 1 | Enable/disable CSV brew logging |
| `I2C_MASTER_SDA_IO` | 11 | ESP32-S3 SDA pin |
| `I2C_MASTER_SCL_IO` | 10 | ESP32-S3 SCL pin |
| `ARDUINO_I2C_ADDRESS` | 0x08 | Arduino Uno I2C slave address |
| `I2C_MASTER_FREQ_HZ` | 50 000 | I2C clock speed (slow/robust for long wires) |
| `TB_SLEEP_THRESHOLD_C` | 25.0 | Thermoblock temp below which display sleeps |
| `TB_WAKE_THRESHOLD_C` | 26.0 | Thermoblock temp above which display wakes (1 °C hysteresis) |
| `TB_OFF_THRESHOLD_C` | 40.0 | Safety limit: schedule wake only fires below this temp |
| `SLEEP_I2C_POLL_MS` | 250 | I2C poll interval when sleeping (4 Hz) |
| `ACTIVE_I2C_POLL_MS` | 100 | I2C poll interval when active (10 Hz) |
| `MAX_LOG_SIZE_BYTES` | 5 MB | Log file rotation threshold |

---

### `main/profile_manager.c/.h` — Brew Profiles & Schedule

Manages up to 3 brew profiles and one weekly schedule, stored as JSON files on
the SD card.  Provides linear interpolation of pressure and temperature curves
during a shot.

#### Key Structs

```c
// One interpolation node in a pressure or temperature curve.
typedef struct {
    float time_s;   // Seconds from shot start
    float value;    // Pressure (bar) or temperature (°C)
} profile_point_t;

// Complete brew profile: targets, pre-infusion config, and up to 20 curve nodes
// for both pressure and temperature.
typedef struct {
    int     id;
    char    name[32];
    char    description[64];
    float   preheat_temperature_c;   // Target temp before shot starts
    uint32_t duration_seconds;
    float   target_temperature_c;
    float   target_pressure_bar;
    preinfusion_config_t preinfusion;
    profile_point_t pressure_points[20];
    int     num_pressure_points;
    profile_point_t temp_points[20];
    int     num_temp_points;
    bool    is_valid;
} brew_profile_t;

// Per-day auto-wake entry.  check_schedule_wake() in main.c triggers a power-button
// pulse within a 15-minute window of hour:minute.
typedef struct {
    char    day_name[16];
    int     day_index;   // 0=Sunday … 6=Saturday
    bool    enabled;
    uint8_t hour;
    uint8_t minute;
} schedule_entry_t;
```

#### Public Functions

| Function | Description |
|----------|-------------|
| `profile_manager_init()` | Loads profiles and schedule from SD; falls back to hardcoded defaults if files are absent |
| `load_profiles_from_sd()` | Parses `/sdcard/profiles.json` into `g_profiles[]` |
| `load_schedule_from_sd()` | Parses `/sdcard/schedule.json` into `g_schedule` |
| `save_profiles_to_sd()` | Writes `g_profiles[]` to SD as JSON |
| `save_schedule_to_sd()` | Writes `g_schedule` to SD as JSON |
| `get_target_pressure(id, elapsed_s)` | Linearly interpolates the pressure curve for profile `id` at `elapsed_s` seconds |
| `get_target_temperature(id, elapsed_s)` | Same for the temperature curve |
| `get_profile(id)` | Returns a read-only pointer to a `brew_profile_t` |
| `get_schedule_for_day(day_index)` | Returns the `schedule_entry_t` for the given day (0=Sunday) |
| `validate_profile(profile)` | Checks curve constraints against `g_constraints` |
| `load_default_profiles()` | Populates `g_profiles[]` with hardcoded fallback values |
| `load_default_schedule()` | Populates `g_schedule` with hardcoded fallback values |
| `get_profile_name(index)` | Returns the name string for profile at `index` |

---

### `main/LVGL_UI/LVGL_Coffee_UI.c/.h` — Display UI

LVGL-based UI layer.  Converts `ArduinoMachineState` data into visible labels and
a pressure progress bar on the 360×360 round display.

All pixel coordinates are hardcoded — LVGL's alignment system cannot account for the
circular bezel, so positions are tuned manually per widget.

#### Public Functions

| Function | Description |
|----------|-------------|
| `Lvgl_Coffee_UI_Init()` | Creates all LVGL widgets and loads the background image asset |
| `Lvgl_Coffee_UI_Update()` | Forces an immediate UI refresh from the current Arduino state |
| `Lvgl_Coffee_UI_Loop()` | Periodic callback (~10 Hz from main loop): copies Arduino state and updates labels/bar |

---

### `main/time_sync.c/.h` — Time Management

Manages the relationship between ESP32 system time (software) and the PCF85063
hardware RTC.  On boot, reads the RTC to seed the system clock before WiFi
connects.  After NTP sync, writes back to the RTC only if drift exceeds 5 minutes
(preserves RTC write cycles).

| Function | Description |
|----------|-------------|
| `time_init()` | Boot sequence: reads RTC → sets system time |
| `wifi_init_sta()` | Initialises WiFi station mode |
| `sync_time_from_ntp()` | Configures SNTP and waits for sync |
| `read_time_from_rtc()` | Reads PCF85063 and sets system clock |
| `set_time_manually(y,m,d,h,min,s)` | Manually sets both system time and RTC |
| `update_rtc_from_system()` | Called by Wireless.c after NTP sync; writes to RTC if drift > 5 min |

---

### `main/LCD_Driver/ST77916.c/.h` — Display Driver

Initialises the ST77916 LCD controller over 4-line QSPI and provides the pixel
write function used by the LVGL flush callback.

| Function | Description |
|----------|-------------|
| `LCD_Init()` | Full initialisation sequence: backlight, QSPI, ST77916 panel |
| `ST77916_Init()` | Sends the ST77916 init command sequence via esp-lcd |
| `LCD_addWindow(x0,y0,x1,y1,color)` | Writes a block of 16-bit pixels directly to the panel |
| `Backlight_Init()` | Configures LEDC timer and channel for PWM backlight control |
| `Set_Backlight(0–100)` | Sets backlight brightness as a percentage |

---

### `main/LVGL_Driver/LVGL_Driver.c/.h` — LVGL Integration

Registers the LCD panel as an LVGL display driver and provides the tick timer
callback that drives LVGL's internal time reference.

| Function | Description |
|----------|-------------|
| `LVGL_Init()` | Allocates draw buffer (`LVGL_BUF_LEN` = W×H/20 pixels), registers display driver |
| `example_lvgl_flush_cb(drv, area, map)` | LVGL flush callback: calls `LCD_addWindow` to push rendered pixels |
| `example_increase_lvgl_tick(arg)` | esp_timer callback: increments LVGL tick by `EXAMPLE_LVGL_TICK_PERIOD_MS` (2 ms) |

---

### `main/I2C_Driver/I2C_Driver.c/.h` — I2C Master Wrapper

Thin wrapper around the ESP-IDF I2C driver, used by peripheral drivers (PCF85063,
TCA9554PWR).  The main application task uses the ESP-IDF API directly for Arduino
communication to avoid lock contention.

| Function | Description |
|----------|-------------|
| `I2C_Init()` | Installs the I2C master driver at 400 kHz on SDA=11, SCL=10 |
| `I2C_Write(addr, reg, data, len)` | Writes `len` bytes to a register on device `addr` |
| `I2C_Read(addr, reg, data, len)` | Reads `len` bytes from a register on device `addr` |

---

### `main/PCF85063/PCF85063.c/.h` — Hardware RTC

Driver for the PCF85063A real-time clock connected over I2C.  Provides date/time
read/write and alarm support.  Used by `time_sync.c` to persist time across reboots.

```c
typedef struct {
    uint16_t year;
    uint8_t  month, day, dotw, hour, minute, second;
} datetime_t;
```

| Function | Description |
|----------|-------------|
| `PCF85063_Init()` | Initialises the RTC; clears oscillator stop flag |
| `PCF85063_Set_All(time)` | Writes complete datetime to RTC |
| `PCF85063_Read_Time(time)` | Reads current datetime from RTC |
| `PCF85063_Set_Alarm(time)` | Programs the alarm registers |
| `PCF85063_Get_Alarm_Flag()` | Returns non-zero if alarm has fired |
| `PCF85063_Loop()` | Called by background_drivers_task every 1 s to keep time fresh |
| `datetime_to_str(buf, time)` | Formats a `datetime_t` to a human-readable string |

---

### `main/EXIO/TCA9554PWR.c/.h` — GPIO Expander

Driver for the TCA9554PWR 8-bit I2C GPIO expander (address 0x20).  Used to drive
the LCD reset line and any other signals not directly wired to ESP32-S3 GPIO.

| Function | Description |
|----------|-------------|
| `TCA9554PWR_Init(pinState)` | Sets all pin directions via CONFIG register |
| `EXIO_Init()` | Full system init (calls TCA9554PWR_Init) |
| `Set_EXIO(pin, state)` | Sets a single output pin high or low |
| `Read_EXIO(pin)` | Reads a single input pin |
| `Mode_EXIO(pin, mode)` | Sets a single pin as output (0) or input (1) |

---

### `main/SD_Card/SD_MMC.c/.h` — SD Card

Initialises the SD card via SD-MMC and provides file helpers.  Used by `main.c`
for brew logging and by `profile_manager.c` for JSON profile storage.

| Function | Description |
|----------|-------------|
| `SD_Init()` | Mounts the FAT filesystem on the SD card at `/sdcard` |
| `Open_File(path)` | Opens a file for reading; returns `FILE*` |
| `s_example_write_file(path, data)` | Writes a string to a file (overwrites) |
| `s_example_read_file(path)` | Reads and logs a file to UART |
| `Folder_retrieval(dir, ext, names, max)` | Lists files with a given extension in a directory |

---

### `main/Wireless/Wireless.c/.h` — WiFi & BLE

Manages WiFi station connection, BLE scanning, and NTP time synchronisation.
Reads WiFi credentials from `/sdcard/config.json` at startup.  Calls
`update_rtc_from_system()` as the SNTP callback when time syncs.

| Function | Description |
|----------|-------------|
| `Wireless_Init()` | Loads WiFi config from SD, initialises WiFi station, starts SNTP |
| `wifi_set_reconnect_policy(enable)` | Enables or disables automatic WiFi reconnection |
| `BLE_Scan()` | Performs a BLE GAP scan and returns the number of devices found |

---

## I2C Protocol (ESP32-S3 ↔ Arduino)

- **Bus:** I2C, 50 kHz (low speed chosen for robustness on internal wiring)
- **Arduino slave address:** `0x08`
- **Poll rate:** 10 Hz when active, 4 Hz when display is sleeping
- **Read:** ESP32 requests `sizeof(ArduinoMachineState)` bytes — Arduino replies with
  the current machine state struct
- **Write — command:** one-byte command code followed by optional payload
  - `0x20` — simulate power-button press
- **Write — debug telemetry:** `I2CDebugMessage` struct (type byte + length byte +
  up to 29 characters of text)
- **Connection health:** after 5 consecutive read failures, `is_arduino_connected()`
  returns false and the UI connection indicator turns red
