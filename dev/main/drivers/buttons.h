#ifndef BUTTONS_H
#define BUTTONS_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Button identifiers
 */
typedef enum {
    BUTTON_LEFT = 0,
    BUTTON_RIGHT,
    BUTTON_COUNT
} button_id_t;

/**
 * @brief Combined button event types (button + action)
 */
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_LEFT_SHORT,
    BUTTON_LEFT_LONG,
    BUTTON_RIGHT_SHORT,
    BUTTON_RIGHT_LONG
} button_event_t;

/**
 * @brief Button event callback type
 */
typedef void (*button_callback_t)(button_event_t event);

/**
 * @brief Initialize button handling
 * @param callback Function to call on button events
 * @return ESP_OK on success
 */
esp_err_t buttons_init(button_callback_t callback);

/**
 * @brief Check if a button is currently pressed
 * @param button Button to check
 * @return true if pressed
 */
bool buttons_is_pressed(button_id_t button);

#endif // BUTTONS_H
