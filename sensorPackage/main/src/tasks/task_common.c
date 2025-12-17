/**
 * @file task_common.c
 * @brief Common definitions and shared state for sensor package tasks
 */

#include "tasks/task_common.h"
#include "drivers/lora_protocol.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "task_common";

// NVS namespace and keys for config storage
#define NVS_NAMESPACE "sensor_cfg"
#define NVS_KEY_INTERVAL "interval"
#define NVS_KEY_FLAGS "flags"

// =============================================================================
// Shared Semaphores
// =============================================================================

SemaphoreHandle_t g_config_mutex = NULL;
SemaphoreHandle_t g_lora_mutex = NULL;
SemaphoreHandle_t g_lightning_mutex = NULL;

// =============================================================================
// Shared State
// =============================================================================

sensor_config_t g_current_config;
lightning_data_t g_accumulated_lightning = {0};
volatile uint8_t g_last_weather_sequence = 0;
volatile bool g_awaiting_weather_ack = false;
volatile bool g_tasks_running = false;
int64_t g_start_time_us = 0;
volatile uint8_t g_sensor_error_flags = 0;

// =============================================================================
// LED Signal Flags
// =============================================================================

volatile bool g_led_flash_packet_sent = false;
volatile bool g_led_flash_packet_failed = false;
volatile bool g_led_flash_packet_received = false;
volatile bool g_led_using_fake_data = false;

// =============================================================================
// Statistics
// =============================================================================

volatile uint32_t g_packets_sent = 0;
volatile uint32_t g_packets_acked = 0;
volatile uint32_t g_packets_failed = 0;

// =============================================================================
// Functions
// =============================================================================

esp_err_t task_common_init(void)
{
    ESP_LOGI(TAG, "Initializing shared task resources...");
    
    // Create config mutex
    if (g_config_mutex == NULL) {
        g_config_mutex = xSemaphoreCreateMutex();
        if (g_config_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create config mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Create LoRa mutex
    if (g_lora_mutex == NULL) {
        g_lora_mutex = xSemaphoreCreateMutex();
        if (g_lora_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create LoRa mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Create lightning mutex
    if (g_lightning_mutex == NULL) {
        g_lightning_mutex = xSemaphoreCreateMutex();
        if (g_lightning_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create lightning mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // Initialize default config first
    sensor_config_t default_config = SENSOR_CONFIG_DEFAULT();
    memcpy(&g_current_config, &default_config, sizeof(sensor_config_t));
    ESP_LOGW(TAG, "=== LOADING CONFIG FROM NVS ===");
    
    // Try to load config from NVS
    esp_err_t nvs_err = task_config_load();
    ESP_LOGW(TAG, "=== NVS LOAD RESULT: %s, interval=%d ===", 
             esp_err_to_name(nvs_err), g_current_config.update_interval_sec);
    
    // Initialize lightning data
    lightning_data_init(&g_accumulated_lightning);
    
    // Record start time
    g_start_time_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "Shared task resources initialized");
    return ESP_OK;
}

void task_common_deinit(void)
{
    if (g_config_mutex != NULL) {
        vSemaphoreDelete(g_config_mutex);
        g_config_mutex = NULL;
    }
    
    if (g_lora_mutex != NULL) {
        vSemaphoreDelete(g_lora_mutex);
        g_lora_mutex = NULL;
    }
    
    if (g_lightning_mutex != NULL) {
        vSemaphoreDelete(g_lightning_mutex);
        g_lightning_mutex = NULL;
    }
}

void task_signal_packet_sent(void)
{
    g_led_flash_packet_sent = true;
}

void task_signal_packet_failed(void)
{
    g_led_flash_packet_failed = true;
}

void task_signal_packet_received(void)
{
    g_led_flash_packet_received = true;
}

uint32_t task_get_uptime_sec(void)
{
    int64_t now_us = esp_timer_get_time();
    return (uint32_t)((now_us - g_start_time_us) / 1000000);
}

// =============================================================================
// NVS Config Persistence
// =============================================================================

esp_err_t task_config_load(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No saved config found in NVS: %s (using defaults)", esp_err_to_name(err));
        return err;
    }
    
    // Load interval
    uint16_t interval = 0;
    err = nvs_get_u16(handle, NVS_KEY_INTERVAL, &interval);
    ESP_LOGI(TAG, "NVS read interval: err=%s, value=%d", esp_err_to_name(err), interval);
    if (err == ESP_OK && interval >= MIN_UPDATE_INTERVAL_SEC && interval <= MAX_UPDATE_INTERVAL_SEC) {
        g_current_config.update_interval_sec = interval;
    }
    
    // Load flags (adaptive_power and high_power packed into one byte)
    uint8_t flags = 0;
    err = nvs_get_u8(handle, NVS_KEY_FLAGS, &flags);
    ESP_LOGI(TAG, "NVS read flags: err=%s, value=0x%02X", esp_err_to_name(err), flags);
    if (err == ESP_OK) {
        g_current_config.adaptive_power = (flags & 0x01) != 0;
        g_current_config.high_power = (flags & 0x02) != 0;
    }
    
    nvs_close(handle);
    
    ESP_LOGI(TAG, "Loaded config from NVS: interval=%ds, adaptive=%d, high_power=%d",
             g_current_config.update_interval_sec,
             g_current_config.adaptive_power,
             g_current_config.high_power);
    
    return ESP_OK;
}

esp_err_t task_config_save(void)
{
    ESP_LOGI(TAG, "Saving config: interval=%ds, adaptive=%d, high_power=%d",
             g_current_config.update_interval_sec,
             g_current_config.adaptive_power,
             g_current_config.high_power);
    
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save interval
    ESP_LOGI(TAG, "Writing interval=%d to NVS...", g_current_config.update_interval_sec);
    err = nvs_set_u16(handle, NVS_KEY_INTERVAL, g_current_config.update_interval_sec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save interval: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }
    
    // Save flags
    uint8_t flags = 0;
    if (g_current_config.adaptive_power) flags |= 0x01;
    if (g_current_config.high_power) flags |= 0x02;
    
    ESP_LOGI(TAG, "Writing flags=0x%02X to NVS...", flags);
    err = nvs_set_u8(handle, NVS_KEY_FLAGS, flags);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save flags: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }
    
    // Commit
    ESP_LOGI(TAG, "Committing NVS...");
    err = nvs_commit(handle);
    nvs_close(handle);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "NVS commit SUCCESS - config saved!");
    } else {
        ESP_LOGE(TAG, "NVS commit FAILED: %s", esp_err_to_name(err));
    }
    
    return err;
}
