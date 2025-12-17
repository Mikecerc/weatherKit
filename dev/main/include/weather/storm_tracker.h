#ifndef STORM_TRACKER_H
#define STORM_TRACKER_H

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Storm risk levels
 */
typedef enum {
    STORM_RISK_LOW = 0,
    STORM_RISK_MODERATE,
    STORM_RISK_HIGH,
    STORM_RISK_SEVERE
} storm_risk_t;

/**
 * @brief Lightning strike data
 */
typedef struct {
    float distance_km;      // Distance in km
    int64_t timestamp;      // When strike was detected (ms since boot)
} lightning_strike_t;

/**
 * @brief Storm tracker status
 */
typedef struct {
    storm_risk_t risk_level;
    float pressure_change_3h;       // hPa change over 3 hours
    float humidity_change_3h;       // % change over 3 hours
    int lightning_count_1h;         // Strikes in last hour
    float closest_strike_km;        // Closest strike distance
    bool storm_approaching;         // True if conditions worsening
} storm_status_t;

/**
 * @brief Initialize storm tracker
 * @return ESP_OK on success
 */
esp_err_t storm_tracker_init(void);

/**
 * @brief Record a weather data point
 * @param temp_c Temperature in Celsius
 * @param humidity Humidity in %
 * @param pressure_hpa Pressure in hPa
 */
void storm_tracker_record_weather(float temp_c, float humidity, float pressure_hpa);

/**
 * @brief Record a lightning strike
 * @param distance_km Distance in km
 */
void storm_tracker_record_lightning(float distance_km);

/**
 * @brief Get current storm status
 * @param status Pointer to status structure to fill
 */
void storm_tracker_get_status(storm_status_t *status);

/**
 * @brief Get recent lightning strikes for display
 * @param strikes Array to fill with strike data
 * @param max_count Maximum strikes to return
 * @return Number of strikes returned
 */
int storm_tracker_get_recent_strikes(lightning_strike_t *strikes, int max_count);

/**
 * @brief Clear all lightning data
 */
void storm_tracker_clear_lightning(void);

#endif // STORM_TRACKER_H
