/**
 * @file aht20.h
 * @brief Thin wrapper API for the AHT20 temperature & humidity sensor
 *
 * This header exposes a small, stable API used throughout the project.
 * The implementation in `main/drivers/aht20.c` is a thin wrapper that
 * delegates to the upstream esp-idf `aht20` component when available.
 *
 * The wrapper intentionally does not expose or require I2C setup details;
 * the underlying component or platform initialization is expected to be
 * performed elsewhere in the application.
 */

#ifndef AHT20_H
#define AHT20_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief AHT20 sensor reading
 */
typedef struct {
    float temperature_c;    /**< Temperature in Celsius */
    float humidity_rh;      /**< Relative humidity in % */
    bool valid;             /**< true if reading is valid */
} aht20_data_t;

/**
 * @brief Initialize the AHT20 wrapper/driver
 *
 * The concrete driver may rely on the esp-idf `aht20` component. This
 * function initializes any state required by the wrapper. It does not
 * perform low-level bus initialization itself.
 *
 * @return ESP_OK on success, an esp_err_t otherwise
 */
esp_err_t aht20_init(void);

/**
 * @brief Read latest temperature and humidity from the sensor
 *
 * @param[out] data Pointer to a pre-allocated aht20_data_t to receive values
 * @return ESP_OK on success, an esp_err_t otherwise
 */
esp_err_t aht20_read(aht20_data_t *data);

#endif // AHT20_H
