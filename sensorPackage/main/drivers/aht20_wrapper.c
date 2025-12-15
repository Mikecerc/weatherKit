/**
 * @file ah20.c
 * @brief AHT20 temperature & humidity driver adapted to project I2C API
 *
 * Implements: aht20_init, aht20_deinit, aht20_reinit, aht20_read, aht20_reset,
 * and aht20_is_present. Uses i2c_master_bus_add_device / transmit / transmit_receive
 * functions already used by other drivers in this project.
 */

#include "aht20_wrapper.h"
#include <aht20.h>  // Upstream driver header
#include "esp_log.h"
#include "drivers/i2c_init.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// Try to use upstream aht20 component if available. The upstream component
// typically provides a device handle and simple api. We don't depend on exact
// upstream internals here; instead call the common esp-idf-lib style API when
// present. If the component/header isn't available, the wrapper falls back to
// returning an error so callers can continue to compile.

static const char *TAG = "aht20_wrapper";

// Keep wrapper state minimal. Upstream components often provide their own
// handle types; we use void* here to avoid requiring a specific header.
static bool aht20_wrapper_inited = false;
// Stored upstream device handle returned by aht20_new_sensor()
static aht20_dev_handle_t s_aht20_dev = NULL;
// Managed i2c_bus handle used to (re)create the device on errors
static i2c_bus_handle_t s_managed_bus = NULL;

// Forward declarations for optional upstream functions. If the real header is
// present these will be resolved at link time; otherwise we provide weak
// stubs below so code compiles but returns ESP_ERR_NOT_SUPPORTED.

// Note: many community aht20 components expose functions named like
// `aht20_init()`, `aht20_read()` or `aht20_create()` etc. Our public header
// (`drivers/aht20.h`) defines the expected `aht20_init()` and `aht20_read()`
// signatures used by the rest of this project, so we'll implement those and
// delegate to upstream where available.

// Simple helper to format and copy values from upstream reading to local struct
static void populate_local_from_upstream(aht20_data_t *dst, float temp_c, float hum_rh)
{
        if (!dst) return;
        dst->temperature_c = temp_c;
        dst->humidity_rh = hum_rh;
        dst->valid = true;
}

esp_err_t aht20_init(void)
{
        if (!i2c_bus_is_initialized()) {
                ESP_LOGW(TAG, "I2C bus not initialized, cannot init AHT20");
                return ESP_ERR_INVALID_STATE;
        }

                /* Obtain or create the managed i2c_bus_handle_t wrapper. The managed
                 * i2c_bus component expects an i2c_bus_handle_t (not the lower level
                 * i2c_master_bus_handle_t). Call i2c_bus_create() with a matching
                 * i2c_config_t so the managed component will detect the already
                 * initialized master bus and return a valid bus handle. */
                i2c_config_t bus_conf = {
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
        #if !CONFIG_I2C_BUS_BACKWARD_CONFIG
                        .mode = I2C_MODE_MASTER,
                        .sda_io_num = PIN_I2C_SDA,
                        .scl_io_num = PIN_I2C_SCL,
                        .sda_pullup_en = true,
                        .scl_pullup_en = true,
                        .master = { .clk_speed = I2C_FREQ_HZ },
                        .clk_flags = 0,
        #endif
        #endif
                };

                        i2c_bus_handle_t managed_bus = i2c_bus_create(I2C_NUM_0, &bus_conf);
                        if (managed_bus == NULL) {
                                ESP_LOGW(TAG, "AHT20: failed to obtain managed i2c_bus handle");
                                return ESP_FAIL;
                        }

                        s_managed_bus = managed_bus;

                        aht20_i2c_config_t i2c_conf = {
                                .bus_inst = s_managed_bus,
                                .i2c_addr = I2C_ADDR_AHT20,
                        };

                        esp_err_t ret = aht20_new_sensor(&i2c_conf, &s_aht20_dev);
        if (ret == ESP_OK) {
                aht20_wrapper_inited = true;
                ESP_LOGI(TAG, "AHT20: delegated init to upstream driver");
                return ESP_OK;
        }

        // Upstream driver not available — mark as not initialized and return
        ESP_LOGW(TAG, "AHT20: upstream driver not available, wrapper disabled (%s)", esp_err_to_name(ret));
        aht20_wrapper_inited = false;
        s_aht20_dev = NULL;
        return ret;
}

esp_err_t aht20_read(aht20_data_t *data)
{
        if (!data) return ESP_ERR_INVALID_ARG;

        // Zero the data by default
        memset(data, 0, sizeof(*data));

        // If upstream read is available, use it
                if (!aht20_wrapper_inited || s_aht20_dev == NULL) {
                        ESP_LOGW(TAG, "AHT20: not initialized");
                        return ESP_ERR_INVALID_STATE;
                }

                float temp, hum;
                uint32_t temp_raw, hum_raw;

                const int max_attempts = 2;
                esp_err_t ret = ESP_FAIL;
                for (int attempt = 0; attempt < max_attempts; ++attempt) {
                        // Probe the device before attempting read to avoid unnecessary transfers
                        if (s_managed_bus) {
                                uint8_t addrs[16] = {0};
                                uint8_t found = i2c_bus_scan(s_managed_bus, addrs, sizeof(addrs));
                                bool present = false;
                                for (uint8_t k = 0; k < found; ++k) {
                                        if (addrs[k] == I2C_ADDR_AHT20) { present = true; break; }
                                }
                                if (!present) {
                                        ESP_LOGW(TAG, "AHT20 not present on bus before read attempt %d", attempt + 1);
                                        ret = ESP_ERR_NOT_FOUND;
                                        // attempt recovery if we still have retries
                                        if (attempt + 1 < max_attempts) {
                                                esp_err_t rec = i2c_bus_recover();
                                                ESP_LOGI(TAG, "AHT20: attempted I2C bus recover -> %s", esp_err_to_name(rec));
                                        }
                                        continue;
                                }

                                /* Master-level probe using the internal bus handle to get a direct
                                 * ack check before attempting higher-level reads. This helps
                                 * distinguish managed-scan success from actual device ACK. */
                                i2c_master_bus_handle_t internal = i2c_bus_get_internal_bus_handle(s_managed_bus);
                                if (internal) {
                                        esp_err_t p = i2c_master_probe(internal, I2C_ADDR_AHT20, 50);
                                        ESP_LOGI(TAG, "AHT20 master probe -> %s", esp_err_to_name(p));
                                        if (p != ESP_OK) {
                                                ret = ESP_ERR_NOT_FOUND;
                                                if (attempt + 1 < max_attempts) {
                                                        esp_err_t rec = i2c_bus_recover();
                                                        ESP_LOGI(TAG, "AHT20: attempted I2C bus recover -> %s", esp_err_to_name(rec));
                                                }
                                                continue;
                                        }
                                        // Short settle delay after probe to avoid immediate NACKs
                                        vTaskDelay(pdMS_TO_TICKS(20));
                                }
                        }

                        ret = aht20_read_temperature_humidity(s_aht20_dev, &temp_raw, &temp, &hum_raw, &hum);
                        if (ret == ESP_OK) {
                                populate_local_from_upstream(data, temp, hum);
                                ESP_LOGI(TAG, "AHT20: T=%.2f°C RH=%.2f%% (from upstream)", data->temperature_c, data->humidity_rh);
                                return ESP_OK;
                        }

                        ESP_LOGW(TAG, "AHT20 read attempt %d failed: %s", attempt + 1, esp_err_to_name(ret));

                        // Try to recover the I2C bus and reinitialise the device handle once
                        if (attempt + 1 < max_attempts) {
                                if (s_managed_bus) {
                                        esp_err_t rec = i2c_bus_recover();
                                        ESP_LOGI(TAG, "AHT20: attempted I2C bus recover -> %s", esp_err_to_name(rec));
                                }

                                // Recreate the aht20 device handle (delete old and create new)
                                if (s_aht20_dev) {
                                        aht20_del_sensor(s_aht20_dev);
                                        s_aht20_dev = NULL;
                                }

                                aht20_i2c_config_t i2c_conf = {
                                        .bus_inst = s_managed_bus,
                                        .i2c_addr = I2C_ADDR_AHT20,
                                };
                                esp_err_t newret = aht20_new_sensor(&i2c_conf, &s_aht20_dev);
                                ESP_LOGI(TAG, "AHT20: re-created device handle -> %s", esp_err_to_name(newret));
                                if (newret != ESP_OK) {
                                        // If re-creation failed, break early
                                        ret = newret;
                                        break;
                                }
                                // Longer delay before retry to allow device to settle
                                vTaskDelay(pdMS_TO_TICKS(250));
                        }
                }

                ESP_LOGW(TAG, "AHT20: upstream read not available (%s)", esp_err_to_name(ret));
                return ret;
}
