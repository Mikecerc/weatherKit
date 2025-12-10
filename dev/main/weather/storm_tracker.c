#include "storm_tracker.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

static const char *TAG = "storm_tracker";

// Configuration
#define WEATHER_HISTORY_SIZE    36      // 3 hours at 5-minute intervals
#define LIGHTNING_HISTORY_SIZE  50      // Max recent strikes to track
#define WEATHER_RECORD_INTERVAL_MS (5 * 60 * 1000)  // 5 minutes

// Weather history point
typedef struct {
    float temperature;
    float humidity;
    float pressure;
    int64_t timestamp;
    bool valid;
} weather_point_t;

// Internal state
static weather_point_t weather_history[WEATHER_HISTORY_SIZE];
static int weather_history_index = 0;
static int weather_history_count = 0;

static lightning_strike_t lightning_history[LIGHTNING_HISTORY_SIZE];
static int lightning_history_index = 0;
static int lightning_history_count = 0;

static SemaphoreHandle_t tracker_mutex = NULL;

/**
 * @brief Get current time in milliseconds
 */
static int64_t get_time_ms(void)
{
    return esp_timer_get_time() / 1000;
}

/**
 * @brief Calculate storm risk based on conditions
 */
static storm_risk_t calculate_risk(float pressure_change, float humidity_change, 
                                    int lightning_count, float closest_strike)
{
    int risk_score = 0;
    
    // Pressure dropping fast = higher risk
    if (pressure_change < -6.0f) risk_score += 3;
    else if (pressure_change < -3.0f) risk_score += 2;
    else if (pressure_change < -1.0f) risk_score += 1;
    
    // Humidity rising = higher risk
    if (humidity_change > 20.0f) risk_score += 2;
    else if (humidity_change > 10.0f) risk_score += 1;
    
    // Lightning activity
    if (lightning_count > 20) risk_score += 3;
    else if (lightning_count > 10) risk_score += 2;
    else if (lightning_count > 3) risk_score += 1;
    
    // Close lightning
    if (closest_strike >= 0 && closest_strike < 5.0f) risk_score += 3;
    else if (closest_strike >= 0 && closest_strike < 15.0f) risk_score += 2;
    else if (closest_strike >= 0 && closest_strike < 30.0f) risk_score += 1;
    
    // Convert score to risk level
    if (risk_score >= 8) return STORM_RISK_SEVERE;
    if (risk_score >= 5) return STORM_RISK_HIGH;
    if (risk_score >= 2) return STORM_RISK_MODERATE;
    return STORM_RISK_LOW;
}

esp_err_t storm_tracker_init(void)
{
    ESP_LOGI(TAG, "Initializing storm tracker...");
    
    tracker_mutex = xSemaphoreCreateMutex();
    if (tracker_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_FAIL;
    }
    
    // Clear history
    memset(weather_history, 0, sizeof(weather_history));
    memset(lightning_history, 0, sizeof(lightning_history));
    
    ESP_LOGI(TAG, "Storm tracker initialized");
    return ESP_OK;
}

void storm_tracker_record_weather(float temp_c, float humidity, float pressure_hpa)
{
    if (xSemaphoreTake(tracker_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    int64_t now = get_time_ms();
    
    // Check if enough time has passed since last record
    if (weather_history_count > 0) {
        int last_idx = (weather_history_index - 1 + WEATHER_HISTORY_SIZE) % WEATHER_HISTORY_SIZE;
        if (weather_history[last_idx].valid) {
            int64_t elapsed = now - weather_history[last_idx].timestamp;
            if (elapsed < WEATHER_RECORD_INTERVAL_MS) {
                // Update the most recent record instead
                weather_history[last_idx].temperature = temp_c;
                weather_history[last_idx].humidity = humidity;
                weather_history[last_idx].pressure = pressure_hpa;
                weather_history[last_idx].timestamp = now;
                xSemaphoreGive(tracker_mutex);
                return;
            }
        }
    }
    
    // Add new record
    weather_history[weather_history_index].temperature = temp_c;
    weather_history[weather_history_index].humidity = humidity;
    weather_history[weather_history_index].pressure = pressure_hpa;
    weather_history[weather_history_index].timestamp = now;
    weather_history[weather_history_index].valid = true;
    
    weather_history_index = (weather_history_index + 1) % WEATHER_HISTORY_SIZE;
    if (weather_history_count < WEATHER_HISTORY_SIZE) {
        weather_history_count++;
    }
    
    xSemaphoreGive(tracker_mutex);
}

void storm_tracker_record_lightning(float distance_km)
{
    if (xSemaphoreTake(tracker_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    lightning_history[lightning_history_index].distance_km = distance_km;
    lightning_history[lightning_history_index].timestamp = get_time_ms();
    
    lightning_history_index = (lightning_history_index + 1) % LIGHTNING_HISTORY_SIZE;
    if (lightning_history_count < LIGHTNING_HISTORY_SIZE) {
        lightning_history_count++;
    }
    
    ESP_LOGI(TAG, "Lightning recorded: %.1f km", distance_km);
    
    xSemaphoreGive(tracker_mutex);
}

void storm_tracker_get_status(storm_status_t *status)
{
    if (status == NULL) return;
    
    memset(status, 0, sizeof(storm_status_t));
    status->closest_strike_km = -1;
    
    if (xSemaphoreTake(tracker_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    int64_t now = get_time_ms();
    int64_t three_hours_ago = now - (3 * 60 * 60 * 1000);
    int64_t one_hour_ago = now - (60 * 60 * 1000);
    
    // Calculate weather changes over 3 hours
    float oldest_pressure = 0, oldest_humidity = 0;
    float newest_pressure = 0, newest_humidity = 0;
    bool have_old = false, have_new = false;
    
    for (int i = 0; i < weather_history_count; i++) {
        weather_point_t *point = &weather_history[i];
        if (!point->valid) continue;
        
        if (point->timestamp >= three_hours_ago && !have_old) {
            oldest_pressure = point->pressure;
            oldest_humidity = point->humidity;
            have_old = true;
        }
        
        // Keep updating newest (last valid point)
        newest_pressure = point->pressure;
        newest_humidity = point->humidity;
        have_new = true;
    }
    
    if (have_old && have_new) {
        status->pressure_change_3h = newest_pressure - oldest_pressure;
        status->humidity_change_3h = newest_humidity - oldest_humidity;
    }
    
    // Count lightning in last hour and find closest
    status->lightning_count_1h = 0;
    status->closest_strike_km = -1;
    
    for (int i = 0; i < lightning_history_count; i++) {
        lightning_strike_t *strike = &lightning_history[i];
        if (strike->timestamp >= one_hour_ago) {
            status->lightning_count_1h++;
            if (status->closest_strike_km < 0 || 
                strike->distance_km < status->closest_strike_km) {
                status->closest_strike_km = strike->distance_km;
            }
        }
    }
    
    // Determine if storm is approaching (pressure falling, lightning getting closer)
    status->storm_approaching = (status->pressure_change_3h < -2.0f) && 
                                 (status->lightning_count_1h > 0);
    
    // Calculate overall risk
    status->risk_level = calculate_risk(
        status->pressure_change_3h,
        status->humidity_change_3h,
        status->lightning_count_1h,
        status->closest_strike_km
    );
    
    xSemaphoreGive(tracker_mutex);
}

int storm_tracker_get_recent_strikes(lightning_strike_t *strikes, int max_count)
{
    if (strikes == NULL || max_count <= 0) return 0;
    
    if (xSemaphoreTake(tracker_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return 0;
    }
    
    int64_t now = get_time_ms();
    int64_t one_hour_ago = now - (60 * 60 * 1000);
    int count = 0;
    
    for (int i = 0; i < lightning_history_count && count < max_count; i++) {
        lightning_strike_t *strike = &lightning_history[i];
        if (strike->timestamp >= one_hour_ago) {
            strikes[count++] = *strike;
        }
    }
    
    xSemaphoreGive(tracker_mutex);
    return count;
}

void storm_tracker_clear_lightning(void)
{
    if (xSemaphoreTake(tracker_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;
    }
    
    memset(lightning_history, 0, sizeof(lightning_history));
    lightning_history_index = 0;
    lightning_history_count = 0;
    
    xSemaphoreGive(tracker_mutex);
    ESP_LOGI(TAG, "Lightning history cleared");
}
