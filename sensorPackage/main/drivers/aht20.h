#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float temperature_c;
    float humidity_rh;
    bool valid;
} aht20_data_t;

/**
 * @brief Initialize the AHT20 sensor
 * Attaches to the existing I2C bus.
 */
esp_err_t aht20_init(void);

/**
 * @brief Read temperature and humidity
 */
esp_err_t aht20_read(aht20_data_t *data);