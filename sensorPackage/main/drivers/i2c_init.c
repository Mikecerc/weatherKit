/**
 * @file i2c_init.c
 * @brief I2C bus initialization implementation
 */

#include "i2c_init.h"
#include "esp_log.h"

static const char *TAG = "i2c_init";
static bool i2c_initialized = false;

esp_err_t i2c_bus_init(void)
{
    if (i2c_initialized) {
        ESP_LOGW(TAG, "I2C bus already initialized");
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_HOST, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_HOST, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
        return err;
    }

    i2c_initialized = true;
    ESP_LOGI(TAG, "I2C bus initialized (SDA=%d, SCL=%d, freq=%dHz)", 
             PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
    
    return ESP_OK;
}

esp_err_t i2c_bus_deinit(void)
{
    if (!i2c_initialized) {
        return ESP_OK;
    }

    esp_err_t err = i2c_driver_delete(I2C_HOST);
    if (err == ESP_OK) {
        i2c_initialized = false;
        ESP_LOGI(TAG, "I2C bus deinitialized");
    }
    
    return err;
}

bool i2c_bus_is_initialized(void)
{
    return i2c_initialized;
}