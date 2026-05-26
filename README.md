# Breville Profusion — ESP32-S3 Coffee Machine Firmware

## Overview

ESP-IDF firmware for an ESP32-S3 that controls and displays the UI of a
Breville Profusion espresso machine. Renders a 360×360 round LCD (ST77916
controller) using LVGL 8.3, showing live pressure, group-head temperature,
thermoblock temperature, and brew mode/status.

## Hardware

| Component         | Details                                                      |
|-------------------|--------------------------------------------------------------|
| MCU               | ESP32-S3                                                     |
| Display           | 360×360 round LCD, ST77916 controller                        |
| Display Interface | 4-line QSPI (SCK=40, D0=46, D1=45, D2=42, D3=41, CS=21)     |
| Backlight         | PWM via GPIO 5 (LEDC, 13-bit, configurable in `settings.h`) |
| RTC               | PCF85063 via I2C                                             |
| GPIO Expander     | TCA9554PWR via I2C (drives LCD reset among other things)     |
| SD Card           | SD-MMC for CSV brew logging                                  |
| Arduino Slave     | I2C addr 0x08 — sensor data, PID control of pump/heater     |

## Architecture

Two-MCU design:

- **ESP32-S3** (this firmware) — display, WiFi, SD logging, brew profiles,
  scheduling, and power management. Polls the Arduino slave via I2C.
- **Arduino Uno slave** — reads pressure/temperature sensors; drives pump and
  thermoblock via PID. Build with PlatformIO (`platformio.ini`).

All I2C operations are serialised through a single FreeRTOS queue in `main.c`,
preventing bus contention between the LVGL render path and other callers
(profile manager, scheduler) that need to write commands to the Arduino.

## Features

- **Brew profiles** — up to 3 profiles, each with a 20-point pressure/temp
  curve over time, stored as JSON on the SD card
- **Schedule-based auto wake-up** — per day of week, with a 15-minute trigger window
- **Sleep/wake power management** — based on thermoblock temperature with
  hysteresis (thresholds in `settings.h`)
- **SD card logging** — CSV file of every brew session, auto-rotated at 5 MB
- **WiFi + NTP** — syncs system time; falls back to PCF85063 RTC if offline
- **BLE scanner** — device discovery
- **LVGL 8.3 UI** on 360×360 round display

## Build (ESP32-S3 firmware)

Requires ESP-IDF v5.x.

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

## Configuration

All user-tunable constants — pin assignments, sleep/wake thresholds, poll
intervals, log path — are in `main/settings.h`.

## Project Structure

```
main/                   Application entry point and core logic (main.c)
main/LVGL_UI/           LVGL-based coffee machine UI
main/LCD_Driver/        ST77916 display driver + esp-lcd panel driver
main/LVGL_Driver/       LVGL display registration and tick driver
main/I2C_Driver/        I2C master wrapper
main/PCF85063/          PCF85063 RTC driver
main/SD_Card/           SD-MMC initialisation and file helpers
main/Wireless/          WiFi station init, BLE scan, NTP sync
platformio.ini          Arduino Uno slave build (PlatformIO/AVR)
lib/RBDdimmer/          PWM dimmer library for Arduino slave
```

## See Also

`ARCHITECTURE.md` — full function reference and data-flow description.
