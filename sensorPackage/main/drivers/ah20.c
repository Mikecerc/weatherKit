/**
 * @file ah20.c
 * @brief AHT20 temperature & humidity driver adapted to project I2C API
 *
 * Implements: aht20_init, aht20_deinit, aht20_reinit, aht20_read, aht20_reset,
 * and aht20_is_present. Uses i2c_master_bus_add_device / transmit / transmit_receive
 * functions already used by other drivers in this project.
 */

#include "ah20.h"
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

// Timing
#define AHT20_MEASURE_DELAY_MS  80
#define AHT20_RESET_DELAY_MS    20
#define AHT20_TIMEOUT_MS        1000

static bool aht20_initialized = false;
static i2c_master_dev_handle_t aht20_dev_handle = NULL;

// CRC-8 (poly 0x31) used by some AHT/AT sensors; implement generic CRC check
static uint8_t aht20_calc_crc(const uint8_t *data, uint8_t len)
{
        uint8_t crc = 0xFF;
        for (uint8_t i = 0; i < len; i++) {
                crc ^= data[i];
                for (uint8_t bit = 0; bit < 8; bit++) {
                        if (crc & 0x80) crc = (crc << 1) ^ 0x31;
                        else crc <<= 1;
                }
        }
        return crc;
}

static esp_err_t aht20_write(const uint8_t *buf, size_t len)
{
        if (!aht20_dev_handle) return ESP_ERR_INVALID_STATE;
        return i2c_master_transmit(aht20_dev_handle, (uint8_t *)buf, len, AHT20_TIMEOUT_MS);
}

static esp_err_t aht20_write_cmd(uint8_t cmd, const uint8_t *data, size_t data_len)
{
        uint8_t tx[8];
        tx[0] = cmd;
        if (data_len) memcpy(&tx[1], data, data_len);
        return aht20_write(tx, 1 + data_len);
}

static esp_err_t aht20_read_bytes(uint8_t *out, size_t len)
{
        if (!aht20_dev_handle) return ESP_ERR_INVALID_STATE;
        // read without sending register address (device will stream bytes)
        return i2c_master_receive(aht20_dev_handle, out, len, AHT20_TIMEOUT_MS);
}

// Wait small delay helper
static void aht20_delay_ms(uint32_t ms)
{
        vTaskDelay(pdMS_TO_TICKS(ms));
}

esp_err_t aht20_init(void)
{
        if (!i2c_bus_is_initialized()) {
                ESP_LOGE(TAG, "I2C bus not initialized");
                return ESP_ERR_INVALID_STATE;
        }

        if (aht20_initialized) return ESP_OK;

        ESP_LOGI(TAG, "Initializing AHT20...");

                i2c_device_config_t dev_cfg = {
                        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                        .device_address = I2C_ADDR_AHT20,
                        .scl_speed_hz = I2C_FREQ_HZ,
                };

                // Try both common AHT20 addresses (0x38 default, 0x77 if SDO pulled high)
                const uint8_t candidate_addrs[] = { I2C_ADDR_AHT20, 0x77 };
                esp_err_t err = ESP_ERR_NOT_FOUND;
                for (size_t i = 0; i < sizeof(candidate_addrs); i++) {
                        dev_cfg.device_address = candidate_addrs[i];
                        // Probe the bus first to avoid generating NACKs during transmit
                        if (i2c_master_probe(i2c_bus_get_handle(), dev_cfg.device_address, 100) != ESP_OK) {
                                ESP_LOGD(TAG, "No device at 0x%02X", dev_cfg.device_address);
                                continue;
                        }

                        err = i2c_master_bus_add_device(i2c_bus_get_handle(), &dev_cfg, &aht20_dev_handle);
                        if (err != ESP_OK) {
                                ESP_LOGW(TAG, "Failed to add AHT20 device at 0x%02X: %s", dev_cfg.device_address, esp_err_to_name(err));
                                aht20_dev_handle = NULL;
                                continue;
                        }

                        // Try initialization command per datasheet; if it fails, fall back to a quick measurement probe
                        uint8_t init_payload[3] = {0x08, 0x00, 0x00};
                        esp_err_t init_err = aht20_write_cmd(AHT20_CMD_INIT, init_payload, sizeof(init_payload));
                        if (init_err == ESP_OK) {
                                aht20_delay_ms(50);
                                err = ESP_OK;
                                break;
                        }

                        // Init failed — attempt a simple measurement read to see if device responds (some modules skip init)
                        aht20_delay_ms(50);
                        uint8_t probe_buf[7] = {0};
                        esp_err_t probe_err = i2c_master_receive(aht20_dev_handle, probe_buf, sizeof(probe_buf), AHT20_TIMEOUT_MS);
                        if (probe_err == ESP_OK) {
                                ESP_LOGI(TAG, "AHT20 at 0x%02X responded to measurement probe (skipping init)", dev_cfg.device_address);
                                err = ESP_OK;
                                break;
                        }

                        // Nothing worked at this address — remove device handle and try next
                        i2c_master_bus_rm_device(aht20_dev_handle);
                        aht20_dev_handle = NULL;
                        err = init_err;
                }

                if (err != ESP_OK) {
                        ESP_LOGW(TAG, "AHT20 not initialized (no responsive address found or init failed)");
                        return err;
                }
        aht20_initialized = true;
        ESP_LOGI(TAG, "AHT20 initialized");
        return ESP_OK;
}

esp_err_t aht20_deinit(void)
{
        if (!aht20_initialized || !aht20_dev_handle) return ESP_OK;
        esp_err_t err = i2c_master_bus_rm_device(aht20_dev_handle);
        if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to remove AHT20 device: %s", esp_err_to_name(err));
                return err;
        }
        aht20_dev_handle = NULL;
        aht20_initialized = false;
        return ESP_OK;
}

esp_err_t aht20_reinit(void)
{
        esp_err_t err = aht20_deinit();
        if (err != ESP_OK) return err;
        return aht20_init();
}

esp_err_t aht20_reset(void)
{
        if (!aht20_dev_handle) return ESP_ERR_INVALID_STATE;
        esp_err_t err = aht20_write_cmd(AHT20_CMD_SOFT_RESET, NULL, 0);
        if (err == ESP_OK) aht20_delay_ms(AHT20_RESET_DELAY_MS);
        return err;
}

bool aht20_is_present(void)
{
        if (!i2c_bus_is_initialized()) return false;
        return (i2c_master_probe(i2c_bus_get_handle(), I2C_ADDR_AHT20, 100) == ESP_OK);
}

esp_err_t aht20_read(aht20_data_t *data)
{
        if (!data) return ESP_ERR_INVALID_ARG;
        if (!aht20_initialized || !aht20_dev_handle) return ESP_ERR_INVALID_STATE;

        memset(data, 0, sizeof(*data));

        // Trigger measurement: 0xAC, payload 0x33 0x00 recommended by many examples
        uint8_t payload[2] = {0x33, 0x00};
        esp_err_t err = aht20_write_cmd(AHT20_CMD_TRIGGER, payload, sizeof(payload));
        if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to send measure cmd: %s", esp_err_to_name(err));
                return err;
        }

        aht20_delay_ms(AHT20_MEASURE_DELAY_MS);

                // Read full 7 bytes (status + 6 data + crc) in a single receive
                uint8_t buf[7] = {0};
                err = i2c_master_receive(aht20_dev_handle, buf, sizeof(buf), AHT20_TIMEOUT_MS);
                if (err != ESP_OK) {
                        ESP_LOGE(TAG, "I2C receive failed: %s", esp_err_to_name(err));
                        return err;
                }

                // If busy bit is set in status byte (buf[0]), sensor not ready
                if (buf[0] & 0x80) {
                        ESP_LOGD(TAG, "AHT20 busy after trigger");
                        return ESP_ERR_NOT_FINISHED;
                }

        // Some modules place status in first byte; validate CRC if present
        // We'll accept either 6 or 7 bytes: if CRC matches assume 7-byte layout
        bool crc_ok = false;
        if (sizeof(buf) == 7) {
                uint8_t calc = aht20_calc_crc(buf, 6);
                if (calc == buf[6]) crc_ok = true;
        }

        // Parse per datasheet: humidity is first 20 bits (buf[1..3] upper nibble), temp is next 20 bits
        uint32_t hum_raw = ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | buf[3];
        hum_raw >>= 4; // top 20 bits
        uint32_t temp_raw = ((uint32_t)(buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];

        float humidity = (float)hum_raw * 100.0f / 1048576.0f;
        float temperature = (float)temp_raw * 200.0f / 1048576.0f - 50.0f;

        data->humidity_rh = humidity;
        data->temperature_c = temperature;
        data->valid = true;

        if (!crc_ok) {
                ESP_LOGW(TAG, "AHT20 CRC missing or invalid; values parsed without CRC verification");
        }

        return ESP_OK;
}
