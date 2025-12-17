#include "aht20.h"
#include "i2c_init.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h> // for memset

static const char *TAG = "aht20";

// AHT20 Constants
#define AHT20_ADDR          0x38
#define AHT20_CMD_TRIGGER   0xAC
#define AHT20_CMD_CALIBRATE 0xBE
#define AHT20_STATUS_BUSY   0x80

// Internal State
static i2c_master_dev_handle_t s_aht20_handle = NULL;
static bool s_is_initialized = false;

static void populate_data(aht20_data_t *dst, float temp, float hum)
{
    if (dst) {
        dst->temperature_c = temp;
        dst->humidity_rh = hum;
        dst->valid = true;
    }
}

// Internal function to perform the raw I2C transaction
static esp_err_t internal_aht20_read_raw(float *temp, float *hum)
{
    if (s_aht20_handle == NULL) return ESP_ERR_INVALID_STATE;

    uint8_t write_buf[3] = {AHT20_CMD_TRIGGER, 0x33, 0x00};
    uint8_t read_buf[6] = {0};

    // 1. Send Trigger
    esp_err_t ret = i2c_master_transmit(s_aht20_handle, write_buf, sizeof(write_buf), -1);
    if (ret != ESP_OK) return ret;

    // 2. Wait 80ms for measurement
    vTaskDelay(pdMS_TO_TICKS(80));

    // 3. Read Data
    ret = i2c_master_receive(s_aht20_handle, read_buf, 6, -1);
    if (ret != ESP_OK) return ret;

    // 4. Parse (Bitwise math from datasheet)
    uint32_t hum_raw = ((uint32_t)read_buf[1] << 12) | ((uint32_t)read_buf[2] << 4) | (read_buf[3] >> 4);
    uint32_t temp_raw = ((uint32_t)(read_buf[3] & 0x0F) << 16) | ((uint32_t)read_buf[4] << 8) | read_buf[5];

    *hum = (float)hum_raw * 100.0f / 1048576.0f;
    *temp = ((float)temp_raw * 200.0f / 1048576.0f) - 50.0f;

    return ESP_OK;
}

// --- Public API ---

esp_err_t aht20_init(void)
{
    i2c_master_bus_handle_t bus_handle = i2c_bus_get_handle();

    if (bus_handle == NULL) {
        ESP_LOGE(TAG, "I2C Bus not initialized. Call i2c_bus_init() first.");
        return ESP_ERR_INVALID_STATE;
    }

    // Check if already added
    if (s_aht20_handle != NULL) {
        return ESP_OK; 
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT20_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_aht20_handle);

    if (ret == ESP_OK) {
        s_is_initialized = true;
        ESP_LOGI(TAG, "AHT20 Device added to bus");
    } else {
        ESP_LOGE(TAG, "Failed to add AHT20 device: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t aht20_read(aht20_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    
    // Clear data initially
    memset(data, 0, sizeof(*data));
    data->valid = false;

    if (!s_is_initialized || s_aht20_handle == NULL) {
        // Try lazy initialization
        if (aht20_init() != ESP_OK) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    float temp = 0.0f;
    float hum = 0.0f;
    esp_err_t ret = ESP_FAIL;
    const int max_attempts = 2; // As requested in your example

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        
        ret = internal_aht20_read_raw(&temp, &hum);

        if (ret == ESP_OK) {
            populate_data(data, temp, hum);
            // Optional: Log like the example did
            // ESP_LOGI(TAG, "Read success: T=%.2f H=%.2f", temp, hum);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "AHT20 read attempt %d failed: %s", attempt + 1, esp_err_to_name(ret));
        
        // Small delay before retry
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ret;
}