/**
 * @file temp_humidity_driver.c
 * @brief AHT20 temperature and humidity sensor driver implementation (new I2C API)
 */

#include "temp_humidity_driver.h"
#include "i2c_init.h"
#include "pinout.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "aht20";

// AHT20 Commands
#define AHT20_CMD_INIT          0xBE
#define AHT20_CMD_TRIGGER       0xAC
#define AHT20_CMD_SOFT_RESET    0xBA
#define AHT20_STATUS_BUSY       0x80
#define AHT20_STATUS_CALIBRATED 0x08

// Timing
#define AHT20_MEASURE_DELAY_MS  80
#define AHT20_RESET_DELAY_MS    20
#define AHT20_TIMEOUT_MS        1000

static bool aht20_initialized = false;
static i2c_master_dev_handle_t aht20_dev_handle = NULL;

/**
 * @brief Write command to AHT20
 */
static esp_err_t aht20_write_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    if (!aht20_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Build write buffer: command + optional data
    uint8_t write_buf[4];
    write_buf[0] = cmd;
    if (data && len > 0 && len <= 3) {
        memcpy(&write_buf[1], data, len);
    }
    
    return i2c_master_transmit(aht20_dev_handle, write_buf, 1 + len, AHT20_TIMEOUT_MS);
}

/**
 * @brief Read data from AHT20
 */
static esp_err_t aht20_read_data(uint8_t *data, size_t len)
{
    if (!aht20_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return i2c_master_receive(aht20_dev_handle, data, len, AHT20_TIMEOUT_MS);
}

// Forward declaration
esp_err_t aht20_reset(void);

esp_err_t aht20_init(void)
{
    if (!i2c_bus_is_initialized()) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing AHT20...");
    
    // Add AHT20 device to the I2C bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR_AHT20,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    
    esp_err_t err = i2c_master_bus_add_device(i2c_bus_get_handle(), &dev_config, &aht20_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add AHT20 device: %s", esp_err_to_name(err));
        return err;
    }
    
    // Send soft reset
    err = aht20_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset AHT20: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(aht20_dev_handle);
        aht20_dev_handle = NULL;
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(AHT20_RESET_DELAY_MS));
    
    // Initialize sensor with calibration
    uint8_t init_data[2] = {0x08, 0x00};
    err = aht20_write_cmd(AHT20_CMD_INIT, init_data, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AHT20: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(aht20_dev_handle);
        aht20_dev_handle = NULL;
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Check calibration status
    uint8_t status;
    err = aht20_read_data(&status, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(aht20_dev_handle);
        aht20_dev_handle = NULL;
        return err;
    }
    
    if (!(status & AHT20_STATUS_CALIBRATED)) {
        ESP_LOGW(TAG, "AHT20 not calibrated (status=0x%02X)", status);
    }
    
    aht20_initialized = true;
    ESP_LOGI(TAG, "AHT20 initialized successfully");
    
    return ESP_OK;
}

esp_err_t aht20_deinit(void)
{
    if (aht20_dev_handle) {
        i2c_master_bus_rm_device(aht20_dev_handle);
        aht20_dev_handle = NULL;
    }
    aht20_initialized = false;
    return ESP_OK;
}

esp_err_t aht20_reinit(void)
{
    ESP_LOGI(TAG, "Re-initializing AHT20...");
    aht20_deinit();
    return aht20_init();
}

esp_err_t aht20_read(aht20_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!aht20_initialized || !aht20_dev_handle) {
        ESP_LOGE(TAG, "AHT20 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    data->valid = false;
    
    // Trigger measurement with retry
    uint8_t trigger_data[2] = {0x33, 0x00};
    esp_err_t err = ESP_FAIL;
    
    for (int retry = 0; retry < 3; retry++) {
        err = aht20_write_cmd(AHT20_CMD_TRIGGER, trigger_data, 2);
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "Trigger retry %d: %s", retry + 1, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger measurement: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(AHT20_MEASURE_DELAY_MS));
    
    // Read measurement data (7 bytes)
    uint8_t raw_data[7];
    err = aht20_read_data(raw_data, 7);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data: %s", esp_err_to_name(err));
        return err;
    }
    
    // Check if busy
    if (raw_data[0] & AHT20_STATUS_BUSY) {
        ESP_LOGW(TAG, "Sensor still busy");
        return ESP_ERR_TIMEOUT;
    }
    
    // Extract humidity (20 bits)
    uint32_t raw_humidity = ((uint32_t)raw_data[1] << 12) | 
                           ((uint32_t)raw_data[2] << 4) | 
                           ((uint32_t)raw_data[3] >> 4);
    
    // Extract temperature (20 bits)
    uint32_t raw_temperature = (((uint32_t)raw_data[3] & 0x0F) << 16) | 
                              ((uint32_t)raw_data[4] << 8) | 
                              ((uint32_t)raw_data[5]);
    
    // Convert to actual values
    data->humidity_rh = ((float)raw_humidity / 1048576.0f) * 100.0f;
    data->temperature_c = ((float)raw_temperature / 1048576.0f) * 200.0f - 50.0f;
    data->valid = true;
    
    ESP_LOGD(TAG, "T=%.1f°C, RH=%.1f%%", data->temperature_c, data->humidity_rh);
    
    return ESP_OK;
}

esp_err_t aht20_reset(void)
{
    esp_err_t err = aht20_write_cmd(AHT20_CMD_SOFT_RESET, NULL, 0);
    if (err == ESP_OK) {
        aht20_initialized = false;
        ESP_LOGI(TAG, "AHT20 reset");
    }
    return err;
}

bool aht20_is_present(void)
{
    if (!i2c_bus_is_initialized()) {
        return false;
    }
    
    // Use bus probe to check if device is present
    return (i2c_master_probe(i2c_bus_get_handle(), I2C_ADDR_AHT20, 100) == ESP_OK);
}