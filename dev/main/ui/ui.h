#ifndef UI_H
#define UI_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Page identifiers
 */
typedef enum {
    UI_PAGE_MAIN = 0,           // Main weather data page
    UI_PAGE_LIGHTNING_MAP,      // Lightning distance visualization
    UI_PAGE_CALCULATED,         // Dew point, heat index, etc.
    UI_PAGE_STORM_TRACKER,      // Storm risk and trends
    UI_PAGE_SENSOR_STATUS,      // Sensor connection status
    UI_PAGE_SETTINGS,           // Settings page
    UI_PAGE_COUNT               // Total number of pages
} ui_page_t;

/**
 * @brief Settings menu items
 */
typedef enum {
    SETTINGS_BRIGHTNESS = 0,
    SETTINGS_UNITS,
    SETTINGS_LORA_TX,
    SETTINGS_ABOUT,
    SETTINGS_COUNT
} settings_item_t;

/**
 * @brief Temperature units
 */
typedef enum {
    UNITS_METRIC = 0,           // Celsius, hPa
    UNITS_IMPERIAL              // Fahrenheit, inHg
} unit_system_t;

/**
 * @brief Weather data structure
 */
typedef struct {
    float temperature;          // Temperature in Celsius
    float humidity;             // Humidity in %
    float pressure;             // Pressure in hPa
    float lightning_dist;       // Lightning distance in km (-1 if none)
    bool sensor_connected;      // Remote sensor status
} weather_data_t;

/**
 * @brief UI settings structure
 */
typedef struct {
    uint8_t brightness;         // 0-100%
    unit_system_t units;        // Metric or Imperial
    bool lora_high_power;       // High power TX (resets to false on boot)
} ui_settings_t;

/**
 * @brief Initialize the UI system
 * @return ESP_OK on success
 */
esp_err_t ui_init(void);

/**
 * @brief Update weather data and refresh display
 * @param data Pointer to weather data
 */
void ui_update_weather(const weather_data_t *data);

/**
 * @brief Cycle through pages or menu items (LEFT short press)
 */
void ui_cycle(void);

/**
 * @brief Select current item or enter context (RIGHT short press)
 */
void ui_select(void);

/**
 * @brief Go back / escape current context (RIGHT long press)
 */
void ui_back(void);

/**
 * @brief Show info/help for current page (LEFT long press)
 */
void ui_show_info(void);

/**
 * @brief Get current page
 * @return Current page identifier
 */
ui_page_t ui_get_current_page(void);

/**
 * @brief Set specific page
 * @param page Page to display
 */
void ui_set_page(ui_page_t page);

/**
 * @brief Force refresh the display
 */
void ui_refresh(void);

/**
 * @brief Check and process pending refresh (call from main loop)
 * Handles deferred UI updates from button presses
 */
void ui_check_refresh(void);

/**
 * @brief Get current settings
 * @return Pointer to settings structure
 */
const ui_settings_t* ui_get_settings(void);

/**
 * @brief Check if LoRa high power TX is enabled
 * @note This setting resets to false on every boot for safety
 * @return true if high power TX is enabled, false for low power
 */
bool ui_is_lora_high_power(void);

/**
 * @brief Save settings to flash (NVS)
 * @return ESP_OK on success
 */
esp_err_t ui_save_settings(void);

/**
 * @brief Load settings from flash (NVS)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not saved before
 */
esp_err_t ui_load_settings(void);

#endif // UI_H
