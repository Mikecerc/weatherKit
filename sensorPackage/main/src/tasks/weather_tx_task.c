/**
 * @file weather_tx_task.c
 * @brief Weather data transmission task for sensor package
 * 
 * Handles periodic sensor reading and transmission to base station.
 * Supports both real sensor data and fake data for testing.
 */

#include "tasks/weather_tx_task.h"
#include "tasks/task_common.h"
#include "drivers/lora.h"
#include "drivers/lora_protocol.h"
#include "drivers/aht20.h"
#include "drivers/pressure_driver.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "weather_tx";

static TaskHandle_t tx_task_handle = NULL;

// Fake sensor state (for simulating realistic data)
static float fake_temp = 22.5f;
static float fake_humidity = 45.0f;
static float fake_pressure = 1013.25f;

// Timing
static int64_t last_weather_send_us = 0;

// =============================================================================
// Sensor Data Generation
// =============================================================================

/**
 * @brief Generate fake but realistic-looking weather data
 */
static void generate_fake_weather_data(weather_payload_t *weather)
{
    // Temperature: varies ±0.1°C per reading
    fake_temp += ((float)(esp_random() % 100) - 50.0f) / 500.0f;
    if (fake_temp < -20.0f) fake_temp = -20.0f;
    if (fake_temp > 50.0f) fake_temp = 50.0f;
    
    // Humidity: varies ±0.5% per reading
    fake_humidity += ((float)(esp_random() % 100) - 50.0f) / 100.0f;
    if (fake_humidity < 0.0f) fake_humidity = 0.0f;
    if (fake_humidity > 100.0f) fake_humidity = 100.0f;
    
    // Pressure: varies ±0.1 hPa per reading
    fake_pressure += ((float)(esp_random() % 100) - 50.0f) / 500.0f;
    if (fake_pressure < 950.0f) fake_pressure = 950.0f;
    if (fake_pressure > 1050.0f) fake_pressure = 1050.0f;
    
    weather->temperature = encode_temperature(fake_temp);
    weather->humidity = encode_humidity(fake_humidity);
    weather->pressure = encode_pressure(fake_pressure);
    
    // Copy lightning data (protected by mutex)
    if (xSemaphoreTake(g_lightning_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&weather->lightning, &g_accumulated_lightning, sizeof(lightning_data_t));
        xSemaphoreGive(g_lightning_mutex);
    }
    
    // Include current config
    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        weather->config.update_interval = g_current_config.update_interval_sec;
        weather->config.tx_power = lora_get_current_power();
        weather->config.flags = 0;
        if (g_current_config.adaptive_power) weather->config.flags |= CFG_ADAPTIVE_POWER;
        if (g_current_config.high_power) weather->config.flags |= CFG_HIGH_POWER;
        xSemaphoreGive(g_config_mutex);
    }
    
    weather->last_base_rssi = lora_get_last_rssi();
    weather->sensor_tx_power = lora_get_current_power();
    weather->error_flags = 0;
    weather->uptime_sec = task_get_uptime_sec();
}

/**
 * @brief Read real weather data from sensors
 */
static void read_real_weather_data(weather_payload_t *weather)
{
    float temperature = 0.0f;
    float humidity = 0.0f;
    float pressure = 0.0f;
    
    g_sensor_error_flags = 0;
    
    // Read AHT20
    aht20_data_t aht20_data = {0};
    esp_err_t err = aht20_read(&aht20_data);
    if (err == ESP_OK && aht20_data.valid) {
        temperature = aht20_data.temperature_c;
        humidity = aht20_data.humidity_rh;
    } else {
        ESP_LOGE(TAG, "AHT20 read failed: %s", esp_err_to_name(err));
        g_sensor_error_flags |= ERR_TEMP_SENSOR | ERR_HUMIDITY_SENSOR;
    }
    
    // Read HX710B
    hx710b_data_t hx710b_data = {0};
    err = hx710b_read(&hx710b_data);
    if (err == ESP_OK && hx710b_data.valid) {
        pressure = hx710b_data.pressure_hpa;
    } else {
        ESP_LOGE(TAG, "HX710B read failed: %s", esp_err_to_name(err));
        g_sensor_error_flags |= ERR_PRESSURE_SENSOR;
    }
    
    weather->temperature = encode_temperature(temperature);
    weather->humidity = encode_humidity(humidity);
    weather->pressure = encode_pressure(pressure);
    
    // Copy lightning data (protected by mutex)
    if (xSemaphoreTake(g_lightning_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&weather->lightning, &g_accumulated_lightning, sizeof(lightning_data_t));
        xSemaphoreGive(g_lightning_mutex);
    }
    
    // Include current config
    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        weather->config.update_interval = g_current_config.update_interval_sec;
        weather->config.tx_power = lora_get_current_power();
        weather->config.flags = 0;
        if (g_current_config.adaptive_power) weather->config.flags |= CFG_ADAPTIVE_POWER;
        if (g_current_config.high_power) weather->config.flags |= CFG_HIGH_POWER;
        xSemaphoreGive(g_config_mutex);
    }
    
    weather->last_base_rssi = lora_get_last_rssi();
    weather->sensor_tx_power = lora_get_current_power();
    weather->error_flags = g_sensor_error_flags;
    weather->uptime_sec = task_get_uptime_sec();
    
    ESP_LOGI(TAG, "Weather: T=%.2f°C, RH=%.2f%%, P=%.2f hPa",
             decode_temperature(weather->temperature),
             decode_humidity(weather->humidity),
             decode_pressure(weather->pressure));
}

/**
 * @brief Log weather data for debugging
 */
static void log_weather_data(const weather_payload_t *weather)
{
    ESP_LOGI(TAG, "======== SENSOR DATA ========");
    ESP_LOGI(TAG, "Temp: %.2f°C, Humidity: %.2f%%, Pressure: %.2f hPa",
             decode_temperature(weather->temperature),
             decode_humidity(weather->humidity),
             decode_pressure(weather->pressure));
    ESP_LOGI(TAG, "Lightning: %d strikes (total: %lu)", 
             weather->lightning.strikes_since_ack, 
             (unsigned long)weather->lightning.total_count);
    ESP_LOGI(TAG, "=============================");
}

// =============================================================================
// Task Implementation
// =============================================================================

/**
 * @brief Weather TX task main loop
 */
static void weather_tx_task(void *arg)
{
    ESP_LOGI(TAG, "Weather TX task started");
    
    last_weather_send_us = 0;
    
    while (g_tasks_running) {
        int64_t now_us = esp_timer_get_time();
        
        // Get current interval
        uint16_t weather_interval_sec;
        bool use_fake = false;
        
        if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            weather_interval_sec = g_current_config.update_interval_sec;
            use_fake = g_current_config.use_fake_data;
            xSemaphoreGive(g_config_mutex);
        } else {
            weather_interval_sec = DEFAULT_UPDATE_INTERVAL_SEC;
        }
        
        // Update LED color mode
        g_led_using_fake_data = use_fake;
        
        // Check if it's time to send weather data
        int64_t weather_interval_us = (int64_t)weather_interval_sec * 1000000LL;
        if ((now_us - last_weather_send_us) >= weather_interval_us) {
            ESP_LOGI(TAG, "Reading sensors...");
            
            weather_payload_t weather = {0};
            
            if (use_fake) {
                generate_fake_weather_data(&weather);
            } else {
                read_real_weather_data(&weather);
            }
            
            // Set sequence number
            g_last_weather_sequence++;
            weather.sequence = g_last_weather_sequence;
            
            log_weather_data(&weather);
            
            // Send via LoRa
            if (lora_is_initialized()) {
                if (xSemaphoreTake(g_lora_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    esp_err_t err = lora_send_weather(&weather);
                    xSemaphoreGive(g_lora_mutex);
                    
                    if (err == ESP_OK) {
                        g_packets_sent++;
                        g_awaiting_weather_ack = true;
                        task_signal_packet_sent();
                        ESP_LOGI(TAG, "Weather packet sent seq=%d, entering RX window", g_last_weather_sequence);
                        
                        // Small delay to let RX task grab mutex and enter receive mode
                        // This is critical - base responds quickly to weather packets
                        vTaskDelay(pdMS_TO_TICKS(5));
                    } else {
                        g_packets_failed++;
                        task_signal_packet_failed();
                        ESP_LOGW(TAG, "LoRa send failed: %s", esp_err_to_name(err));
                    }
                }
            } else {
                ESP_LOGI(TAG, "(LoRa not available - data logged only)");
            }
            
            last_weather_send_us = now_us;
        }
        
        // Sleep until next check
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    ESP_LOGI(TAG, "Weather TX task stopped");
    tx_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t weather_tx_task_start(TaskHandle_t *handle)
{
    if (tx_task_handle != NULL) {
        ESP_LOGW(TAG, "Weather TX task already running");
        return ESP_OK;
    }
    
    BaseType_t ret = xTaskCreate(
        weather_tx_task,
        "weather_tx",
        4096,
        NULL,
        4,  // Medium priority
        &tx_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create weather TX task");
        return ESP_ERR_NO_MEM;
    }
    
    if (handle != NULL) {
        *handle = tx_task_handle;
    }
    
    return ESP_OK;
}

void weather_tx_task_stop(void)
{
    tx_task_handle = NULL;
}
