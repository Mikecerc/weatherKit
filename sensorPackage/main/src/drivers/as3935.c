/**
 * @file as3935_driver.c
 * @brief AS3935 Franklin Lightning Sensor driver implementation (new I2C API)
 */

#include "as3935.h"
#include "i2c_init.h"
#include "pinout.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "as3935";

// AS3935 Register addresses
#define AS3935_REG_AFE_GB           0x00
#define AS3935_REG_PWD              0x00
#define AS3935_REG_NF_LEV           0x01
#define AS3935_REG_WDTH             0x01
#define AS3935_REG_CL_STAT          0x02
#define AS3935_REG_MIN_NUM_LIGH     0x02
#define AS3935_REG_SREJ             0x02
#define AS3935_REG_LCO_FDIV         0x03
#define AS3935_REG_MASK_DIST        0x03
#define AS3935_REG_INT              0x03
#define AS3935_REG_DISTANCE         0x07
#define AS3935_REG_DISP_LCO         0x08
#define AS3935_REG_DISP_SRCO        0x08
#define AS3935_REG_DISP_TRCO        0x08
#define AS3935_REG_TUN_CAP          0x08

// Direct commands
#define AS3935_DIRECT_COMMAND       0x96

// Timing
#define AS3935_TIMEOUT_MS           1000
#define AS3935_CALIB_DELAY_MS       2

static bool as3935_initialized = false;
static uint32_t total_strikes = 0;
static as3935_data_t last_data = {0};
static void (*interrupt_callback)(as3935_interrupt_t) = NULL;
static i2c_master_dev_handle_t as3935_dev_handle = NULL;

/**
 * @brief Write to AS3935 register
 */
static esp_err_t as3935_write_register(uint8_t reg, uint8_t value)
{
    if (!as3935_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_transmit(as3935_dev_handle, write_buf, 2, AS3935_TIMEOUT_MS);
}

/**
 * @brief Read from AS3935 register
 */
static esp_err_t as3935_read_register(uint8_t reg, uint8_t *value)
{
    if (!value) return ESP_ERR_INVALID_ARG;
    if (!as3935_dev_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return i2c_master_transmit_receive(as3935_dev_handle, &reg, 1, value, 1, AS3935_TIMEOUT_MS);
}

/**
 * @brief Modify bits in a register
 */
static esp_err_t as3935_modify_register(uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current;
    esp_err_t err = as3935_read_register(reg, &current);
    if (err != ESP_OK) return err;
    
    uint8_t new_value = (current & ~mask) | (value & mask);
    return as3935_write_register(reg, new_value);
}

/**
 * @brief Initialize IRQ GPIO
 */
static esp_err_t as3935_init_irq(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_AS3935_IRQ),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    
    return gpio_config(&io_conf);
}

esp_err_t as3935_init(const as3935_config_t *config)
{
    if (!i2c_bus_is_initialized()) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Initializing AS3935...");
    
    // Use default config if none provided
    as3935_config_t cfg;
    if (config) {
        cfg = *config;
    } else {
        as3935_config_t default_cfg = AS3935_CONFIG_DEFAULT();
        cfg = default_cfg;
    }
    
    // Add AS3935 device to the I2C bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR_AS3935,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    
    esp_err_t err = i2c_master_bus_add_device(i2c_bus_get_handle(), &dev_config, &as3935_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add AS3935 device: %s", esp_err_to_name(err));
        return err;
    }
    
    // Initialize IRQ pin
    err = as3935_init_irq();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize IRQ pin: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(as3935_dev_handle);
        as3935_dev_handle = NULL;
        return err;
    }
    
    // Reset device using direct command
    err = as3935_write_register(AS3935_DIRECT_COMMAND, 0x96);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset AS3935: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(as3935_dev_handle);
        as3935_dev_handle = NULL;
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(2));
    
    // Power up
    err = as3935_power_up();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to power up: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(as3935_dev_handle);
        as3935_dev_handle = NULL;
        return err;
    }
    
    // Set indoor/outdoor mode
    err = as3935_set_indoor(cfg.indoor);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set indoor mode: %s", esp_err_to_name(err));
    }
    
    // Set noise floor
    err = as3935_set_noise_floor(cfg.noise_floor);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set noise floor: %s", esp_err_to_name(err));
    }
    
    // Set watchdog threshold
    err = as3935_modify_register(AS3935_REG_WDTH, 0x0F, cfg.watchdog_threshold & 0x0F);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set watchdog threshold: %s", esp_err_to_name(err));
    }
    
    // Set minimum number of lightning strikes
    uint8_t min_strikes_val = 0;
    switch (cfg.min_strikes) {
        case 1:  min_strikes_val = 0; break;
        case 5:  min_strikes_val = 1; break;
        case 9:  min_strikes_val = 2; break;
        case 16: min_strikes_val = 3; break;
        default: min_strikes_val = 0; break;
    }
    err = as3935_modify_register(AS3935_REG_MIN_NUM_LIGH, 0x30, min_strikes_val << 4);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set min strikes: %s", esp_err_to_name(err));
    }
    
    // Clear statistics
    err = as3935_clear_statistics();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to clear statistics: %s", esp_err_to_name(err));
    }
    
    // Calibrate oscillator
    err = as3935_calibrate();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to calibrate: %s", esp_err_to_name(err));
    }
    
    as3935_initialized = true;
    total_strikes = 0;
    
    ESP_LOGI(TAG, "AS3935 initialized successfully (indoor=%d, noise_floor=%d)",
             cfg.indoor, cfg.noise_floor);
    
    return ESP_OK;
}

esp_err_t as3935_read_interrupt(as3935_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    if (!as3935_initialized) return ESP_ERR_INVALID_STATE;
    
    data->valid = false;
    
    // Read interrupt source
    uint8_t int_val;
    esp_err_t err = as3935_read_register(AS3935_REG_INT, &int_val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read interrupt: %s", esp_err_to_name(err));
        return err;
    }
    
    data->event = (as3935_interrupt_t)(int_val & 0x0F);
    
    // If lightning detected, read distance and energy
    if (data->event == AS3935_INT_LIGHTNING) {
        // Read distance
        uint8_t distance;
        err = as3935_read_register(AS3935_REG_DISTANCE, &distance);
        if (err == ESP_OK) {
            data->distance_km = distance & 0x3F;
        } else {
            data->distance_km = 0;
        }
        
        // Read energy (registers 0x04, 0x05, 0x06)
        uint8_t energy_bytes[3];
        for (int i = 0; i < 3; i++) {
            err = as3935_read_register(0x04 + i, &energy_bytes[i]);
            if (err != ESP_OK) {
                energy_bytes[i] = 0;
            }
        }
        data->energy = ((uint32_t)energy_bytes[2] << 16) | 
                      ((uint32_t)energy_bytes[1] << 8) | 
                      energy_bytes[0];
        
        total_strikes++;
        data->strike_count = total_strikes;
        data->valid = true;
        
        // Update last data
        last_data = *data;
        
        ESP_LOGI(TAG, "Lightning detected! Distance=%d km, Energy=%lu, Total=%lu",
                 data->distance_km, (unsigned long)data->energy, 
                 (unsigned long)total_strikes);
        
        // Call callback if registered
        if (interrupt_callback) {
            interrupt_callback(data->event);
        }
    } else {
        data->distance_km = 0;
        data->energy = 0;
        data->strike_count = total_strikes;
        
        if (data->event == AS3935_INT_NOISE) {
            ESP_LOGD(TAG, "Noise detected");
        } else if (data->event == AS3935_INT_DISTURBER) {
            ESP_LOGD(TAG, "Disturber detected");
        }
    }
    
    return ESP_OK;
}

esp_err_t as3935_get_statistics(as3935_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;
    
    *data = last_data;
    data->strike_count = total_strikes;
    
    return ESP_OK;
}

esp_err_t as3935_clear_statistics(void)
{
    if (!as3935_initialized) return ESP_ERR_INVALID_STATE;
    
    // Clear statistics register
    esp_err_t err = as3935_modify_register(AS3935_REG_CL_STAT, 0x40, 0x40);
    if (err == ESP_OK) {
        total_strikes = 0;
        memset(&last_data, 0, sizeof(last_data));
        ESP_LOGI(TAG, "Statistics cleared");
    }
    
    return err;
}

esp_err_t as3935_calibrate(void)
{
    if (!as3935_initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Calibrating AS3935...");
    
    // Display oscillator on IRQ pin
    esp_err_t err = as3935_modify_register(AS3935_REG_DISP_SRCO, 0x80, 0x80);
    if (err != ESP_OK) return err;
    
    vTaskDelay(pdMS_TO_TICKS(AS3935_CALIB_DELAY_MS));
    
    // Turn off display
    err = as3935_modify_register(AS3935_REG_DISP_SRCO, 0x80, 0x00);
    
    ESP_LOGI(TAG, "Calibration complete");
    return err;
}

esp_err_t as3935_set_indoor(bool indoor)
{
    if (!as3935_initialized) return ESP_ERR_INVALID_STATE;
    
    // AFE_GB: 0x12 for indoor, 0x0E for outdoor
    uint8_t afe_value = indoor ? 0x12 : 0x0E;
    esp_err_t err = as3935_modify_register(AS3935_REG_AFE_GB, 0x3E, afe_value << 1);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Set to %s mode", indoor ? "indoor" : "outdoor");
    }
    
    return err;
}

esp_err_t as3935_set_noise_floor(uint8_t level)
{
    if (!as3935_initialized) return ESP_ERR_INVALID_STATE;
    if (level > 7) return ESP_ERR_INVALID_ARG;
    
    esp_err_t err = as3935_modify_register(AS3935_REG_NF_LEV, 0x70, level << 4);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Noise floor set to %d", level);
    }
    
    return err;
}

esp_err_t as3935_power_down(void)
{
    if (!as3935_initialized) return ESP_ERR_INVALID_STATE;
    
    esp_err_t err = as3935_modify_register(AS3935_REG_PWD, 0x01, 0x01);
    
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "AS3935 powered down");
    }
    
    return err;
}

esp_err_t as3935_power_up(void)
{
    esp_err_t err = as3935_modify_register(AS3935_REG_PWD, 0x01, 0x00);
    
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));
        ESP_LOGI(TAG, "AS3935 powered up");
    }
    
    return err;
}

bool as3935_is_present(void)
{
    if (!i2c_bus_is_initialized()) {
        return false;
    }
    
    // Use bus probe to check if device is present
    return (i2c_master_probe(i2c_bus_get_handle(), I2C_ADDR_AS3935, 100) == ESP_OK);
}

void as3935_register_interrupt_callback(void (*callback)(as3935_interrupt_t event))
{
    interrupt_callback = callback;
}