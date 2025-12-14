/**
 * @file i2c_diag.c
 * @brief Simple I2C diagnostic task to probe devices and exercise transactions.
 */

#include "i2c_init.h"
#include "pinout.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "i2c_diag";

void i2c_diag_task(void *arg)
{
    ESP_LOGI(TAG, "Starting I2C diagnostics...");
    if (!i2c_bus_is_initialized()) {
        ESP_LOGW(TAG, "I2C bus not initialized");
        vTaskDelete(NULL);
        return;
    }

    i2c_master_bus_handle_t bus = i2c_bus_get_handle();

    const uint8_t addrs[] = { I2C_ADDR_AHT20, 0x77, I2C_ADDR_AS3935 };
    for (size_t i = 0; i < sizeof(addrs); i++) {
        uint8_t a = addrs[i];
        esp_err_t r = i2c_master_probe(bus, a, 200);
        ESP_LOGI(TAG, "probe 0x%02X -> %s", a, esp_err_to_name(r));
    }

    // Attempt a small txrx with AHT20 address if present
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR_AHT20,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    i2c_master_dev_handle_t dev = NULL;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) == ESP_OK && dev) {
        ESP_LOGI(TAG, "Added device handle for 0x%02X", dev_cfg.device_address);

        // Try receive
        uint8_t buf[7] = {0};
        esp_err_t r = i2c_master_receive(dev, buf, sizeof(buf), 200);
        ESP_LOGI(TAG, "receive -> %s", esp_err_to_name(r));
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "raw: %02X %02X %02X %02X %02X %02X %02X", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);
        }

        // Try transmit (init command) - do not treat failure as fatal
        uint8_t init_cmd[4] = { 0xBE, 0x08, 0x00, 0x00 };
        r = i2c_master_transmit(dev, init_cmd, sizeof(init_cmd), 200);
        ESP_LOGI(TAG, "transmit init -> %s", esp_err_to_name(r));

        i2c_master_bus_rm_device(dev);
    } else {
        ESP_LOGW(TAG, "Could not add device handle for 0x%02X", dev_cfg.device_address);
    }

    ESP_LOGI(TAG, "I2C diagnostics complete");
    vTaskDelete(NULL);
}

void start_i2c_diag(void)
{
    xTaskCreatePinnedToCore(i2c_diag_task, "i2c_diag", 2048, NULL, 5, NULL, tskNO_AFFINITY);
}
