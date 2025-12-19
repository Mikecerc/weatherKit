/**
 * @file buzzer.h
 * @brief Simple buzzer driver (on/off) using GPIO
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Initialize buzzer GPIO
 * @return ESP_OK on success
 */
esp_err_t buzzer_init(void);

/**
 * @brief Play a continuous tone using PWM at the requested frequency (Hz)
 * @param freq_hz Frequency in Hertz (e.g. 2000)
 * @return ESP_OK on success
 */
esp_err_t buzzer_play_tone(uint32_t freq_hz);

/**
 * @brief Stop any playing tone
 */
void buzzer_stop(void);

/**
 * @brief Set locate mode for buzzer (start/stop a default multi-tone pattern)
 *
 * When enabled this will run a small background task that quickly alternates
 * between multiple tones to make the device easier to locate.
 *
 * @param enable true to enable locate pattern, false to disable
 */
void buzzer_set_locate(bool enable);

/**
 * @brief Check whether buzzer is currently on
 */
bool buzzer_is_on(void);

#endif // BUZZER_H
