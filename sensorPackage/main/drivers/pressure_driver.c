/*
 * @file hx710b_driver.c
 * @brief HX710B barometric pressure sensor driver implementation
 */

#include "pressure_driver.h"
#include "pinout.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

static const char *TAG = "hx710b";

// Calibration parameters (adjust these based on your sensor)
#define HX710B_SCALE_FACTOR     0.01f   // Convert raw to hPa (adjust after calibration)
#define HX710B_BASELINE         8388608 // 2^23 (midpoint of 24-bit signed value)

static bool hx710b_initialized = false;
static float calibration_offset = 0.0f;

// Timing constants (microseconds)
#define HX710B_CLK_PULSE_US     1       // Clock pulse width
#define HX710B_SETTLE_US        1       // Settling time after clock

esp_err_t hx710b_init(void)
{
    ESP_LOGI(TAG, "Initializing HX710B...");
    
    // Configure CLK pin as output
    gpio_config_t clk_conf = {
        .pin_bit_mask = (1ULL << PIN_SPI_CLK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t err = gpio_config(&clk_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CLK pin: %s", esp_err_to_name(err));
        return err;
    }
    
    // Configure DOUT pin as input
    gpio_config_t dout_conf = {
        .pin_bit_mask = (1ULL << PIN_HX710B_DOUT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    err = gpio_config(&dout_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure DOUT pin: %s", esp_err_to_name(err));
        return err;
    }
    
    // Set CLK low initially
    gpio_set_level(PIN_SPI_CLK, 0);
    
    // Power up the sensor
    err = hx710b_power_up();
    if (err != ESP_OK) {
        return err;
    }
    
    hx710b_initialized = true;
    ESP_LOGI(TAG, "HX710B initialized successfully");
    
    return ESP_OK;
}

bool hx710b_is_ready(void)
{
    if (!hx710b_initialized) {
        return false;
    }
    
    // DOUT goes low when data is ready
    return (gpio_get_level(PIN_HX710B_DOUT) == 0);
}

esp_err_t hx710b_read(hx710b_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!hx710b_initialized) {
        ESP_LOGE(TAG, "HX710B not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    data->valid = false;
    
    // Wait for sensor to be ready (with timeout)
    int timeout = 100; // 100ms timeout
    while (!hx710b_is_ready() && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout--;
    }
    
    if (timeout == 0) {
        ESP_LOGW(TAG, "Timeout waiting for sensor ready");
        return ESP_ERR_TIMEOUT;
    }
    
    // Read 24 bits (MSB first)
    int32_t value = 0;
    
    // Disable interrupts for accurate timing
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    
    for (int i = 0; i < 24; i++) {
        // Clock pulse
        gpio_set_level(PIN_SPI_CLK, 1);
        ets_delay_us(HX710B_CLK_PULSE_US);
        
        // Read bit
        value = (value << 1) | gpio_get_level(PIN_HX710B_DOUT);
        
        gpio_set_level(PIN_SPI_CLK, 0);
        ets_delay_us(HX710B_CLK_PULSE_US);
    }
    
    // One more pulse to set gain (25 pulses = gain 128)
    gpio_set_level(PIN_SPI_CLK, 1);
    ets_delay_us(HX710B_CLK_PULSE_US);
    gpio_set_level(PIN_SPI_CLK, 0);
    
    portEXIT_CRITICAL(&mux);
    
    // Convert from 24-bit two's complement to 32-bit signed
    if (value & 0x800000) {
        value |= 0xFF000000; // Sign extend
    }
    
    data->raw_value = value;
    
    // Convert to pressure (this is a simplified conversion)
    // You'll need to calibrate this based on your specific sensor
    float pressure = ((float)(value - HX710B_BASELINE) * HX710B_SCALE_FACTOR) + 1013.25f;
    pressure += calibration_offset;
    
    // Sanity check (typical atmospheric pressure range)
    if (pressure < 800.0f || pressure > 1200.0f) {
        ESP_LOGW(TAG, "Pressure out of range: %.2f hPa (raw=%ld)", pressure, (long)value);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    data->pressure_hpa = pressure;
    data->valid = true;
    
    ESP_LOGD(TAG, "Pressure=%.2f hPa (raw=%ld)", pressure, (long)value);
    
    return ESP_OK;
}

esp_err_t hx710b_power_down(void)
{
    if (!hx710b_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Hold CLK high for > 60us to enter power down
    gpio_set_level(PIN_SPI_CLK, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    ESP_LOGI(TAG, "HX710B powered down");
    return ESP_OK;
}

esp_err_t hx710b_power_up(void)
{
    // Set CLK low to wake up
    gpio_set_level(PIN_SPI_CLK, 0);
    
    // Wait for sensor to settle
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "HX710B powered up");
    return ESP_OK;
}

void hx710b_set_offset(float offset_hpa)
{
    calibration_offset = offset_hpa;
    ESP_LOGI(TAG, "Calibration offset set to %.2f hPa", offset_hpa);
}