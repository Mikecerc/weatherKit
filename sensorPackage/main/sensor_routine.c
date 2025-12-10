/**
 * @file sensor_routine.c
 * @brief Sensor routine for WeatherKit Sensor Package
 * 
 * Handles periodic sensor reading and transmission to base station.
 * Currently uses fake data - real sensor drivers to be added later.
 */

#include "sdkconfig.h"
#include "sensor_routine.h"
#include "drivers/lora.h"
#include "drivers/lora_protocol.h"
#include "drivers/led.h"
#include "pinout.h"
#include "drivers/lightning_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "sensor";

// =============================================================================
// State Variables
// =============================================================================

static sensor_config_t current_config;
static TaskHandle_t sensor_task_handle = NULL;
static bool is_running = false;
static SemaphoreHandle_t config_mutex = NULL;

// Statistics
static uint32_t packets_sent = 0;
static uint32_t packets_acked = 0;
static uint32_t packets_failed = 0;

// Timing
static int64_t start_time_us = 0;
static int64_t last_weather_send_us = 0;
static int64_t last_heartbeat_send_us = 0;

// Fake sensor state (for simulating realistic data)
static float fake_temp = 22.5f;          // Starting temperature (°C)
static float fake_humidity = 45.0f;      // Starting humidity (%)
static float fake_pressure = 1013.25f;   // Starting pressure (hPa)

// =============================================================================
// Callbacks
// =============================================================================

/**
 * @brief Called when configuration message received from base
 */
static void on_config_received(const config_payload_t *config)
{
    ESP_LOGI(TAG, "Config received: interval=%ds, heartbeat=%ds, power=%d",
             config->update_interval, config->heartbeat_interval, config->tx_power);
    
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Update intervals if valid
        if (config->update_interval >= MIN_UPDATE_INTERVAL_SEC && 
            config->update_interval <= MAX_UPDATE_INTERVAL_SEC) {
            current_config.update_interval_sec = config->update_interval;
        }
        
        if (config->heartbeat_interval > 0) {
            current_config.heartbeat_interval_sec = config->heartbeat_interval;
        }
        
        // Update adaptive power setting
        current_config.adaptive_power = (config->flags & CFG_ADAPTIVE_POWER) != 0;
        lora_set_adaptive_power(current_config.adaptive_power);
        
        // If adaptive power is off, use the specified power level
        if (!current_config.adaptive_power && config->tx_power >= TX_POWER_MIN) {
            lora_set_tx_power(config->tx_power);
        }
        
        xSemaphoreGive(config_mutex);
    }
}

/**
 * @brief Called when ACK received from base
 */
static void on_ack_received(const ack_payload_t *ack)
{
    ESP_LOGD(TAG, "ACK received: seq=%d, rssi=%d, suggested_power=%d",
             ack->acked_sequence, ack->rssi, ack->suggested_power);
    
    packets_acked++;
}

/**
 * @brief Called when ping received from base
 */
static void on_ping_received(uint8_t ping_type)
{
    ESP_LOGI(TAG, "Ping received: type=%d", ping_type);
    
    if (ping_type == 1) {
        // Locate request - flash LED blue
        ESP_LOGI(TAG, "LOCATE request - flashing LED");
        led_flash_locate(3000);  // Flash for 3 seconds
    }
}

// =============================================================================
// Fake Sensor Data Generation
// =============================================================================

/**
 * @brief Generate fake but realistic-looking weather data
 * 
 * Simulates gradual changes with some randomness to look like
 * real sensor readings.
 */
static void generate_fake_weather_data(weather_payload_t *weather)
{
    // Add small random variations to simulate real sensor drift
    // Temperature: varies ±0.1°C per reading, bounded to reasonable range
    fake_temp += ((float)(esp_random() % 100) - 50.0f) / 500.0f;
    if (fake_temp < -20.0f) fake_temp = -20.0f;
    if (fake_temp > 50.0f) fake_temp = 50.0f;
    
    // Humidity: varies ±0.5% per reading, bounded 0-100%
    fake_humidity += ((float)(esp_random() % 100) - 50.0f) / 100.0f;
    if (fake_humidity < 0.0f) fake_humidity = 0.0f;
    if (fake_humidity > 100.0f) fake_humidity = 100.0f;
    
    // Pressure: varies ±0.1 hPa per reading, bounded to reasonable range
    fake_pressure += ((float)(esp_random() % 100) - 50.0f) / 500.0f;
    if (fake_pressure < 950.0f) fake_pressure = 950.0f;
    if (fake_pressure > 1050.0f) fake_pressure = 1050.0f;
    
    // Encode values
    weather->temperature = encode_temperature(fake_temp);
    weather->humidity = encode_humidity(fake_humidity);
    weather->pressure = encode_pressure(fake_pressure);
    
    // No lightning for fake data
    lightning_data_init(&weather->lightning);
    
    // Include link quality info
    weather->rssi = lora_get_last_rssi();
    weather->tx_power = lora_get_current_power();
}

/**
 * @brief Generate sensor status payload
 */
static void generate_status_data(status_payload_t *status_data)
{
    // Fake battery data (would come from ADC in real implementation)
    status_data->battery_mv = 3700 + (esp_random() % 200);  // 3.7-3.9V
    status_data->battery_percent = 85 + (esp_random() % 15); // 85-100%
    
    // Link quality
    status_data->rssi = lora_get_last_rssi();
    status_data->snr = 0;  // TODO: Get from LoRa status
    status_data->tx_power = lora_get_current_power();
    
    // Uptime
    status_data->uptime_min = sensor_routine_get_uptime_min();
    
    // Current configuration
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        status_data->update_interval = current_config.update_interval_sec;
        xSemaphoreGive(config_mutex);
    }
    
    // No errors (fake data always works!)
    status_data->error_flags = 0;
    
    ESP_LOGI(TAG, "Status: battery=%dmV (%d%%), uptime=%d min",
             status_data->battery_mv, status_data->battery_percent, 
             status_data->uptime_min);
}

// =============================================================================
// Sensor Task
// =============================================================================

/**
 * @brief Main sensor routine task
 */
static void sensor_task(void *arg)
{
    ESP_LOGI(TAG, "Sensor task started");
    
    // Initialize timing
    start_time_us = esp_timer_get_time();
    
    // Set last send times to trigger immediate first send
    // By setting them far in the past, the interval check will pass immediately
    last_weather_send_us = start_time_us - (DEFAULT_UPDATE_INTERVAL_SEC * 1000000LL) - 1000000LL;
    last_heartbeat_send_us = start_time_us - (DEFAULT_HEARTBEAT_INTERVAL_SEC * 1000000LL) - 1000000LL;
    
    // Put LoRa in receive mode initially
    lora_receive();
    
    while (is_running) {
        int64_t now_us = esp_timer_get_time();
        
        // Get current intervals
        uint16_t weather_interval_sec;
        uint16_t heartbeat_interval_sec;
        
        if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            weather_interval_sec = current_config.update_interval_sec;
            heartbeat_interval_sec = current_config.heartbeat_interval_sec;
            xSemaphoreGive(config_mutex);
        } else {
            weather_interval_sec = DEFAULT_UPDATE_INTERVAL_SEC;
            heartbeat_interval_sec = DEFAULT_HEARTBEAT_INTERVAL_SEC;
        }
        
        // Check for incoming packets FIRST (config, ACK, ping from base station)
        // This ensures we're responsive to base station commands
        while (lora_received()) {
            ESP_LOGI(TAG, ">>> Packet received from base!");
            lora_process_rx();
            lora_receive();  // Go back to RX mode
        }
        
        // Check if it's time to send weather data
        int64_t weather_interval_us = (int64_t)weather_interval_sec * 1000000LL;
        if ((now_us - last_weather_send_us) >= weather_interval_us) {
            ESP_LOGI(TAG, "Sending weather data...");
            
            weather_payload_t weather = {0};
            generate_fake_weather_data(&weather);
            
            esp_err_t err = lora_send_weather(&weather);
            if (err == ESP_OK) {
                packets_sent++;
                ESP_LOGI(TAG, "Weather packet sent (total=%lu)", (unsigned long)packets_sent);
            } else {
                packets_failed++;
                ESP_LOGW(TAG, "Weather packet send failed: %s", esp_err_to_name(err));
            }
            
            last_weather_send_us = now_us;
            
            // Go back to receive mode to listen for ACK
            lora_receive();
        }
        
        // Check if it's time to send heartbeat/status
        int64_t heartbeat_interval_us = (int64_t)heartbeat_interval_sec * 1000000LL;
        if ((now_us - last_heartbeat_send_us) >= heartbeat_interval_us) {
            ESP_LOGI(TAG, "Sending status heartbeat...");
            
            status_payload_t status_data = {0};
            generate_status_data(&status_data);
            
            esp_err_t err = lora_send_sensor_status(&status_data);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Status packet sent");
            } else {
                ESP_LOGW(TAG, "Status packet send failed: %s", esp_err_to_name(err));
            }
            
            last_heartbeat_send_us = now_us;
            
            // Go back to receive mode
            lora_receive();
        }
        
        // Ensure we're in receive mode
        lora_receive();
        
        // Small delay - sensor spends most time listening for base commands
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_LOGI(TAG, "Sensor task stopped");
    lora_sleep();
    vTaskDelete(NULL);
}

// =============================================================================
// Public API
// =============================================================================

esp_err_t sensor_routine_init(const sensor_config_t *config)
{
    ESP_LOGI(TAG, "Initializing sensor routine...");
    
    // Create mutex for config access
    if (config_mutex == NULL) {
        config_mutex = xSemaphoreCreateMutex();
        if (config_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create config mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Set configuration
    if (config != NULL) {
        memcpy(&current_config, config, sizeof(sensor_config_t));
    } else {
        sensor_config_t default_config = SENSOR_CONFIG_DEFAULT();
        memcpy(&current_config, &default_config, sizeof(sensor_config_t));
    }
    
    // Initialize LED for locate function
    esp_err_t err = led_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to initialize LED: %s (continuing anyway)", esp_err_to_name(err));
        // Don't fail - LED is optional
    }
    
    // Initialize LoRa module
    err = lora_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LoRa: %s", esp_err_to_name(err));
        return err;
    }
    
    // Set up callbacks
    lora_set_config_callback(on_config_received);
    lora_set_ack_callback(on_ack_received);
    lora_set_ping_callback(on_ping_received);
    
    // Enable adaptive power if configured
    lora_set_adaptive_power(current_config.adaptive_power);
    
    // Run diagnostics
    lora_run_diagnostics();
    
    ESP_LOGI(TAG, "Sensor routine initialized (interval=%ds, heartbeat=%ds)",
             current_config.update_interval_sec, current_config.heartbeat_interval_sec);
    
    return ESP_OK;
}

esp_err_t sensor_routine_start(void)
{
    if (is_running) {
        ESP_LOGW(TAG, "Sensor routine already running");
        return ESP_OK;
    }
    
    is_running = true;
    
    // Create sensor task
    BaseType_t ret = xTaskCreate(
        sensor_task,
        "sensor_task",
        4096,           // Stack size
        NULL,
        5,              // Priority
        &sensor_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        is_running = false;
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Sensor routine started");
    return ESP_OK;
}

void sensor_routine_stop(void)
{
    if (!is_running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping sensor routine...");
    is_running = false;
    
    // Wait for task to finish (it will delete itself)
    vTaskDelay(pdMS_TO_TICKS(200));
    sensor_task_handle = NULL;
}

bool sensor_routine_is_running(void)
{
    return is_running;
}

void sensor_routine_update_config(const sensor_config_t *config)
{
    if (config == NULL) return;
    
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&current_config, config, sizeof(sensor_config_t));
        lora_set_adaptive_power(current_config.adaptive_power);
        xSemaphoreGive(config_mutex);
        
        ESP_LOGI(TAG, "Config updated: interval=%ds, heartbeat=%ds",
                 current_config.update_interval_sec, current_config.heartbeat_interval_sec);
    }
}

void sensor_routine_get_config(sensor_config_t *config)
{
    if (config == NULL) return;
    
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(config, &current_config, sizeof(sensor_config_t));
        xSemaphoreGive(config_mutex);
    }
}

void sensor_routine_force_send(void)
{
    ESP_LOGI(TAG, "Force sending weather data...");
    
    weather_payload_t weather = {0};
<<<<<<< HEAD
    //generate_fake_weather_data(&weather);
=======
    generate_fake_weather_data(&weather);
>>>>>>> a79b699e4732d53f924e94b7514f3b0e58771170
    
    esp_err_t err = lora_send_weather(&weather);
    if (err == ESP_OK) {
        packets_sent++;
        ESP_LOGI(TAG, "Weather packet sent");
    } else {
        packets_failed++;
        ESP_LOGW(TAG, "Weather packet send failed: %s", esp_err_to_name(err));
    }
    
    // Return to receive mode
    lora_receive();
}

uint16_t sensor_routine_get_uptime_min(void)
{
    int64_t now_us = esp_timer_get_time();
    int64_t uptime_us = now_us - start_time_us;
    uint32_t uptime_min = (uint32_t)(uptime_us / 60000000LL);
    
    // Clamp to uint16 max
    if (uptime_min > 65535) {
        uptime_min = 65535;
    }
    
    return (uint16_t)uptime_min;
}

void sensor_routine_get_stats(uint32_t *sent, uint32_t *acked, uint32_t *failed)
{
    if (sent) *sent = packets_sent;
    if (acked) *acked = packets_acked;
    if (failed) *failed = packets_failed;
<<<<<<< HEAD
}
=======
}
>>>>>>> a79b699e4732d53f924e94b7514f3b0e58771170
