/**
 * @file LVGL_Coffee_UI.h
 * @brief Breville Profusion User Interface Header
 * @version V46 - Clean Build
 * @date 2025-11-29
 * @author [Your Name/Project Name]
 * * @details
 * This module manages the Graphical User Interface (GUI) for the espresso machine using the LVGL library.
 * It handles the layout of the circular display, including the pressure gauge, temperature readouts,
 * timers, and status indicators.
 * * @note
 * This header is designed for C compatibility (extern "C") to allow integration with C++ logic if needed.
 */

#ifndef LVGL_COFFEE_UI_H
#define LVGL_COFFEE_UI_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the coffee machine user interface.
 * @details Creates the main screen, allocates memory for labels/bars, sets up styles,
 * and loads the background image. Must be called *after* `lv_init()` and `lv_disp_drv_register()`.
 */
void Lvgl_Coffee_UI_Init(void);

/**
 * @brief Force an immediate update of all UI elements.
 * @details Reads the latest `ArduinoMachineState` and refreshes the display widgets.
 * Typically called by the internal loop, but exposed if an external event needs to force a redraw.
 */
void Lvgl_Coffee_UI_Update(void);

/**
 * @brief Periodic UI Loop Callback.
 * @details Should be called inside the main ESP32 `while(1)` loop.
 * Handles timing (e.g., limiting updates to 10Hz) to prevent UI lag or flickering.
 */
void Lvgl_Coffee_UI_Loop(void);

#ifdef __cplusplus
}
#endif

#endif // LVGL_COFFEE_UI_H