/**
 * @file aht20_driver.c
 * @brief AHT20 temperature and humidity sensor driver implementation
 */

#include "i2c_init.h"
#include "pinout.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

/**
 * @brief Write command to AHT20
 */
static esp_err_t aht20_write_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (I2C_ADDR_AHT20 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, cmd, true);
    
    if (data && len > 0) {
        i2c_master_write(cmd_handle, data, len, true);
    }
    
    i2c_master_stop(cmd_handle);
    esp_err_t err = i2c_master_cmd_begin(I2C_HOST, cmd_handle, pdMS_TO_TICKS(AHT20_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd_handle);
    
    return err;
}

/**
 * @brief Read data from AHT20
 */
static esp_err_t aht20_read_data(uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (I2C_ADDR_AHT20 << 1) | I2C_MASTER_READ, true);
    
    if (len > 1) {
        i2c_master_read(cmd_handle, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd_handle, data + len - 1, I2C_MASTER_NACK);
    
    i2c_master_stop(cmd_handle);
    esp_err_t err = i2c_master_cmd_begin(I2C_HOST, cmd_handle, pdMS_TO_TICKS(AHT20_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd_handle);
    
    return err;
}

esp_err_t aht20_init(void)
{
    if (!i2c_bus_is_initialized()) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing AHT20...");
    
    // Send soft reset
    esp_err_t err = aht20_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset AHT20: %s", esp_err_to_name(err));
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(AHT20_RESET_DELAY_MS));
    
    // Initialize sensor with calibration
    uint8_t init_data[2] = {0x08, 0x00};
    err = aht20_write_cmd(AHT20_CMD_INIT, init_data, 2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize AHT20: %s", esp_err_to_name(err));
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Check calibration status
    uint8_t status;
    err = aht20_read_data(&status, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status: %s", esp_err_to_name(err));
        return err;
    }
    
    if (!(status & AHT20_STATUS_CALIBRATED)) {
        ESP_LOGW(TAG, "AHT20 not calibrated (status=0x%02X)", status);
    }
    
    aht20_initialized = true;
    ESP_LOGI(TAG, "AHT20 initialized successfully");
    
    return ESP_OK;
}

esp_err_t aht20_read(aht20_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!aht20_initialized) {
        ESP_LOGE(TAG, "AHT20 not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    data->valid = false;
    
    // Trigger measurement
    uint8_t trigger_data[2] = {0x33, 0x00};
    esp_err_t err = aht20_write_cmd(AHT20_CMD_TRIGGER, trigger_data, 2);
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
    
    uint8_t data;
    esp_err_t err = aht20_read_data(&data, 1);
    return (err == ESP_OK);
}