/**
 * @file i2c_init.c
 * @brief I2C bus initialization implementation (new driver API)
 */

#include "i2c_init.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "i2c_init";
static bool i2c_initialized = false;
static i2c_master_bus_handle_t s_bus_handle = NULL;

esp_err_t i2c_bus_init(void)
{
    if (s_bus_handle != NULL) {
        return ESP_OK; // Already initialized
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = -1, // Auto-select port
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = 1,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &s_bus_handle);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C Master Bus initialized successfully");
    } else {
        ESP_LOGE(TAG, "Failed to init I2C Master Bus: %s", esp_err_to_name(ret));
    }

    return ret;
}

bool i2c_bus_is_initialized(void)
{
    return i2c_initialized;
}

i2c_master_bus_handle_t i2c_bus_get_handle(void)
{
    return s_bus_handle;
}


void i2c_bus_log_scan(void)
{
    if (!i2c_initialized) {
        ESP_LOGW(TAG, "I2C bus not initialized, cannot scan");
        return;
    }
    
    ESP_LOGI(TAG, "Scanning I2C bus...");
    ESP_LOGI(TAG, "     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
    
    int devices_found = 0;
    
    for (int i = 0; i < 128; i += 16) {
        char line[64];
        int pos = sprintf(line, "%02X: ", i);
        
        for (int j = 0; j < 16; j++) {
            uint8_t addr = i + j;
            
            // Skip reserved addresses
            if (addr < 0x03 || addr > 0x77) {
                pos += sprintf(line + pos, "   ");
                continue;
            }
            
            // Probe for device at this address
            esp_err_t ret = i2c_master_probe(s_bus_handle, addr, 50);
            
            if (ret == ESP_OK) {
                pos += sprintf(line + pos, "%02X ", addr);
                devices_found++;
            } else {
                pos += sprintf(line + pos, "-- ");
            }
        }
        ESP_LOGI(TAG, "%s", line);
    }
    
    ESP_LOGI(TAG, "I2C scan complete: %d device(s) found", devices_found);
    
    // Log expected devices
    ESP_LOGI(TAG, "Expected devices:");
    ESP_LOGI(TAG, "  - AHT20 at 0x%02X", I2C_ADDR_AHT20);
    ESP_LOGI(TAG, "  - AS3935 at 0x%02X", I2C_ADDR_AS3935);
}