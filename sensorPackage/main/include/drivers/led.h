/**
 * @file led.h
 * @brief Simple LED driver for status/locate indication
 * 
 * Uses the onboard WS2812 RGB LED on GPIO 21
 */

#ifndef LED_H
#define LED_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Initialize the LED driver
 * @return ESP_OK on success
 */
esp_err_t led_init(void);

/**
 * @brief Set LED color
 * @param red Red component (0-255)
 * @param green Green component (0-255)  
 * @param blue Blue component (0-255)
 */
void led_set_color(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Turn LED off
 */
void led_off(void);

#endif // LED_H
