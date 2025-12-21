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
    SETTINGS_REFRESH_RATE,      // Sensor refresh rate (5-60s)
    SETTINGS_LOCATE,            // Locate mode (LED on/off)
    SETTINGS_HIGH_POWER,        // Send high power setting to sensor
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
    uint8_t lightning_count;    // Lightning strikes since last ACK
    uint32_t lightning_total;   // Total lightning count since sensor power-on
    bool sensor_connected;      // Remote sensor status
    
    // Sensor configuration (echoed from sensor)
    uint16_t sensor_update_interval;  // Sensor update interval in seconds
    uint8_t sensor_tx_power;    // Sensor TX power in dBm
    bool sensor_high_power;     // Sensor high power mode enabled
    bool sensor_adaptive_power; // Sensor adaptive power enabled
    int8_t sensor_rssi;         // RSSI at sensor (of base TX)
    uint32_t sensor_uptime_sec; // Sensor uptime in seconds
    
    // Base station link quality
    int8_t base_rssi;           // RSSI at base (of sensor TX)
    uint8_t base_tx_power;      // Base TX power in dBm
    
    // Timing
    uint32_t last_rx_time_ms;   // Timestamp of last received packet
} weather_data_t;

/**
 * @brief UI settings structure
 */
typedef struct {
    uint8_t brightness;         // 0-100%
    unit_system_t units;        // Metric or Imperial
    uint8_t refresh_rate;       // Sensor refresh rate in seconds (5-60)
    bool locate_enabled;        // Locate LED enabled
    bool sensor_high_power;     // High power mode for sensor TX
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
 * @brief Check if locate mode is enabled
 * @return true if locate LED should be on
 */
bool ui_is_locate_enabled(void);

/**
 * @brief Get current sensor refresh rate setting
 * @return Refresh rate in seconds
 */
uint8_t ui_get_refresh_rate(void);

/**
 * @brief Check if sensor high power is enabled
 * @return true if sensor should use high power TX
 */
bool ui_is_sensor_high_power(void);

/**
 * @brief Enter standby mode (display off, low power, still receiving data)
 */
void ui_enter_standby(void);

/**
 * @brief Exit standby mode (wake display)
 */
void ui_exit_standby(void);

/**
 * @brief Check if in standby mode
 * @return true if in standby
 */
bool ui_is_standby(void);

/**
 * @brief Toggle standby mode
 */
void ui_toggle_standby(void);

/**
 * @brief Check if locate ping should be sent (clears flag after read)
 * @return true if locate ping should be sent to sensor
 */
bool ui_check_locate_pending(void);

/**
 * @brief Check if config update should be sent (clears flag after read)
 * @return true if config should be sent to sensor
 */
bool ui_check_config_pending(void);

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
