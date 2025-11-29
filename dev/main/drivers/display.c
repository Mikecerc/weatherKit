#include "display.h"
#include "pinout.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "display";

// OLED Display Configuration
#define OLED_WIDTH          128
#define OLED_HEIGHT         64

static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t oled_dev = NULL;

// Display buffer
static uint8_t display_buffer[OLED_WIDTH * OLED_HEIGHT / 8];

/**
 * @brief Write command to OLED
 */
static esp_err_t oled_write_cmd(uint8_t cmd)
{
    uint8_t data[2] = {0x00, cmd};  // 0x00 = command mode
    return i2c_master_transmit(oled_dev, data, 2, pdMS_TO_TICKS(100));
}

/**
 * @brief Write data to OLED
 */
static esp_err_t oled_write_data(uint8_t *data, size_t len)
{
    uint8_t *buffer = malloc(len + 1);
    if (!buffer) return ESP_ERR_NO_MEM;
    
    buffer[0] = 0x40;  // 0x40 = data mode
    memcpy(buffer + 1, data, len);
    esp_err_t ret = i2c_master_transmit(oled_dev, buffer, len + 1, pdMS_TO_TICKS(1000));
    free(buffer);
    return ret;
}

/**
 * @brief Initialize I2C and OLED device
 */
static esp_err_t i2c_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C bus...");
    
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_HOST,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR_OLED,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &dev_config, &oled_dev));
    
    return ESP_OK;
}

/**
 * @brief Initialize the SSD1309 OLED panel
 */
static esp_err_t oled_init(void)
{
    ESP_LOGI(TAG, "Initializing OLED panel...");
    
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for display power up
    
    // SSD1309 initialization sequence
    oled_write_cmd(0xAE);  // Display OFF
    
    oled_write_cmd(0xD5);  // Set display clock
    oled_write_cmd(0x80);
    
    oled_write_cmd(0xA8);  // Set multiplex ratio
    oled_write_cmd(0x3F);  // 64 lines
    
    oled_write_cmd(0xD3);  // Set display offset
    oled_write_cmd(0x00);
    
    oled_write_cmd(0x40);  // Set start line to 0
    
    oled_write_cmd(0x20);  // Set memory addressing mode
    oled_write_cmd(0x00);  // Horizontal addressing
    
    oled_write_cmd(0xA1);  // Segment remap (flip horizontal)
    oled_write_cmd(0xC8);  // COM scan direction (flip vertical)
    
    oled_write_cmd(0xDA);  // Set COM pins
    oled_write_cmd(0x12);
    
    oled_write_cmd(0x81);  // Set contrast
    oled_write_cmd(0x7F);
    
    oled_write_cmd(0xD9);  // Set pre-charge period
    oled_write_cmd(0x22);
    
    oled_write_cmd(0xDB);  // Set VCOMH
    oled_write_cmd(0x34);
    
    oled_write_cmd(0xA4);  // Display from RAM
    oled_write_cmd(0xA6);  // Normal display (not inverted)
    
    oled_write_cmd(0xAF);  // Display ON
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    return ESP_OK;
}

/**
 * @brief Clear the display buffer
 */
void display_clear(void)
{
    memset(display_buffer, 0, sizeof(display_buffer));
}

/**
 * @brief Send buffer to display
 */
void display_refresh(void)
{
    // Ensure display start line is 0 (prevents vertical offset issues)
    oled_write_cmd(0x40);  // Set display start line to 0
    
    // Reset to start of display RAM
    oled_write_cmd(0x21);  // Set column address
    oled_write_cmd(0);     // Start column 0
    oled_write_cmd(127);   // End column 127
    
    oled_write_cmd(0x22);  // Set page address  
    oled_write_cmd(0);     // Start page 0
    oled_write_cmd(7);     // End page 7
    
    // Write entire buffer
    oled_write_data(display_buffer, sizeof(display_buffer));
}

/**
 * @brief Set a pixel in the buffer
 */
void display_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    
    if (on) {
        display_buffer[x + (y / 8) * OLED_WIDTH] |= (1 << (y % 8));
    } else {
        display_buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
}

/**
 * @brief Draw a filled rectangle
 */
void display_fill_rect(int x, int y, int w, int h, bool on)
{
    for (int i = x; i < x + w; i++) {
        for (int j = y; j < y + h; j++) {
            display_set_pixel(i, j, on);
        }
    }
}

esp_err_t display_init(void)
{
    esp_err_t ret;
    
    ret = i2c_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        return ret;
    }
    
    ret = oled_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize OLED");
        return ret;
    }
    
    // Clear display
    display_clear();
    display_refresh();
    
    ESP_LOGI(TAG, "Display initialized successfully");
    return ESP_OK;
}

void display_set_brightness(uint8_t brightness)
{
    // SSD1309 contrast command: 0x81 followed by contrast value (0-255)
    // Scale percentage (10-100) to hardware range (25-255)
    uint8_t hw_brightness = (brightness * 255) / 100;
    if (hw_brightness < 25) hw_brightness = 25;  // Minimum visible
    
    oled_write_cmd(0x81);  // Set contrast command
    oled_write_cmd(hw_brightness);
}
