/**
 * @file sensor_routine.c
 * @brief Sensor routine orchestrator for WeatherKit Sensor Package
 * 
 * Initializes hardware and starts/stops the individual tasks:
 * - LED status task
 * - LoRa RX task (receives config, ACKs)
 * - Weather TX task (sends sensor data)
 */

#include "sensor_routine.h"
#include "tasks/task_common.h"
#include "tasks/led_status_task.h"
#include "tasks/lora_rx_task.h"
#include "tasks/weather_tx_task.h"

#include "drivers/lora.h"
#include "drivers/led.h"
#include "drivers/buzzer.h"
#include "drivers/i2c_init.h"
#include "drivers/aht20.h"
#include "drivers/pressure_driver.h"

#include "esp_log.h"
#include <string.h>

static const char *TAG = "sensor";

// Task handles
static TaskHandle_t led_task_handle = NULL;
static TaskHandle_t rx_task_handle = NULL;
static TaskHandle_t tx_task_handle = NULL;

// =============================================================================
// Public API
// =============================================================================

esp_err_t sensor_routine_init(const sensor_config_t *config)
{
    ESP_LOGI(TAG, "Initializing sensor routine...");
    
    // Initialize shared task resources
    esp_err_t err = task_common_init();
    if (err != ESP_OK) {
        return err;
    }
    
    // Apply provided configuration
    if (config != NULL) {
        if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            memcpy(&g_current_config, config, sizeof(sensor_config_t));
            xSemaphoreGive(g_config_mutex);
        }
    }
    
    // Initialize LED
    err = led_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LED init failed: %s (continuing)", esp_err_to_name(err));
    }
    
    // Initialize buzzer
    err = buzzer_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Buzzer init failed: %s (continuing)", esp_err_to_name(err));
    }
    
    // Initialize sensors if using real data
    bool use_fake = false;
    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        use_fake = g_current_config.use_fake_data;
        xSemaphoreGive(g_config_mutex);
    }
    
    if (!use_fake) {
        ESP_LOGI(TAG, "Initializing real sensors...");
        
        err = i2c_bus_init();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
            return err;
        }
        
        i2c_bus_log_scan();
        
        err = aht20_init();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "AHT20 init failed: %s", esp_err_to_name(err));
            g_sensor_error_flags |= ERR_TEMP_SENSOR | ERR_HUMIDITY_SENSOR;
        }
        
        err = hx710b_init();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HX710B init failed: %s", esp_err_to_name(err));
            g_sensor_error_flags |= ERR_PRESSURE_SENSOR;
        }
        
        ESP_LOGI(TAG, "Sensors initialized (errors=0x%02X)", g_sensor_error_flags);
    } else {
        ESP_LOGI(TAG, "Using fake sensor data");
    }
    
    // Initialize LoRa
    err = lora_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LoRa init failed: %s (continuing without wireless)", esp_err_to_name(err));
    } else {
        bool adaptive = false;
        if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            adaptive = g_current_config.adaptive_power;
            xSemaphoreGive(g_config_mutex);
        }
        lora_set_adaptive_power(adaptive);
        lora_run_diagnostics();
    }
    
    ESP_LOGI(TAG, "Sensor routine initialized");
    return ESP_OK;
}

esp_err_t sensor_routine_start(void)
{
    if (g_tasks_running) {
        ESP_LOGW(TAG, "Sensor routine already running");
        return ESP_OK;
    }
    
    g_tasks_running = true;
    
    // Update LED mode before starting
    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_led_using_fake_data = g_current_config.use_fake_data;
        xSemaphoreGive(g_config_mutex);
    }
    
    // Start LED status task
    esp_err_t err = led_status_task_start(&led_task_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LED task start failed (continuing)");
    }
    
    // Start LoRa RX task (if LoRa available)
    if (lora_is_initialized()) {
        err = lora_rx_task_start(&rx_task_handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "LoRa RX task start failed");
        }
    }
    
    // Start weather TX task
    err = weather_tx_task_start(&tx_task_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Weather TX task start failed");
        g_tasks_running = false;
        return err;
    }
    
    ESP_LOGI(TAG, "Sensor routine started");
    return ESP_OK;
}

void sensor_routine_stop(void)
{
    if (!g_tasks_running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping sensor routine...");
    g_tasks_running = false;
    
    // Wait for tasks to exit
    vTaskDelay(pdMS_TO_TICKS(200));
    
    led_task_handle = NULL;
    rx_task_handle = NULL;
    tx_task_handle = NULL;
    
    lora_sleep();
}

bool sensor_routine_is_running(void)
{
    return g_tasks_running;
}

void sensor_routine_update_config(const sensor_config_t *config)
{
    if (config == NULL) return;
    
    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&g_current_config, config, sizeof(sensor_config_t));
        lora_set_adaptive_power(g_current_config.adaptive_power);
        xSemaphoreGive(g_config_mutex);
        
        ESP_LOGI(TAG, "Config updated: interval=%ds, high_power=%d",
                 g_current_config.update_interval_sec, g_current_config.high_power);
    }
}

void sensor_routine_get_config(sensor_config_t *config)
{
    if (config == NULL) return;
    
    if (xSemaphoreTake(g_config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ESP_LOGI(TAG, "get_config: g_current_config.interval=%d", g_current_config.update_interval_sec);
        memcpy(config, &g_current_config, sizeof(sensor_config_t));
        xSemaphoreGive(g_config_mutex);
        ESP_LOGI(TAG, "get_config: copied interval=%d", config->update_interval_sec);
    }
}

uint16_t sensor_routine_get_uptime_min(void)
{
    uint32_t uptime_sec = task_get_uptime_sec();
    uint32_t uptime_min = uptime_sec / 60;
    return (uptime_min > 65535) ? 65535 : (uint16_t)uptime_min;
}

void sensor_routine_get_stats(uint32_t *sent, uint32_t *acked, uint32_t *failed)
{
    if (sent) *sent = g_packets_sent;
    if (acked) *acked = g_packets_acked;
    if (failed) *failed = g_packets_failed;
}
