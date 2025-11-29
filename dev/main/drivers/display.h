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

#endif // DISPLAY_H
