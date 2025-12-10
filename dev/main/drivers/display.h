#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the OLED display
 */
esp_err_t display_init(void);

/**
 * @brief Clear the display buffer
 */
void display_clear(void);

/**
 * @brief Send buffer to display
 */
void display_refresh(void);

/**
 * @brief Set a pixel in the buffer
 */
void display_set_pixel(int x, int y, bool on);

/**
 * @brief Draw a filled rectangle
 */
void display_fill_rect(int x, int y, int w, int h, bool on);

/**
 * @brief Set display brightness/contrast
 * @param brightness 0-255 (will be scaled from 0-100% input)
 */
void display_set_brightness(uint8_t brightness);

/**
 * @brief Put display into sleep mode (very low power ~10µA)
 */
void display_sleep(void);

/**
 * @brief Wake display from sleep mode
 */
void display_wake(void);

/**
 * @brief Check if display is in sleep mode
 * @return true if display is sleeping
 */
bool display_is_sleeping(void);

#endif // DISPLAY_H
