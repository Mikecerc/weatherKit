/**
 * @file led.c
 * @brief Simple LED driver using RMT for WS2812
 */

#include "led.h"
#include "pinout.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "led";

// WS2812 timing (in nanoseconds)
#define WS2812_T0H_NS 350
#define WS2812_T0L_NS 800
#define WS2812_T1H_NS 700
#define WS2812_T1L_NS 600
#define WS2812_RESET_NS 50000

static rmt_channel_handle_t led_channel = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static bool initialized = false;
static bool locate_active = false;

// Current LED color
static uint8_t current_r = 0, current_g = 0, current_b = 0;

// WS2812 encoder for RMT
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_ws2812_encoder_t;

static size_t rmt_encode_ws2812(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                 const void *primary_data, size_t data_size,
                                 rmt_encode_state_t *ret_state)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = ws2812_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = ws2812_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    
    switch (ws2812_encoder->state) {
        case 0: // Send RGB data
            encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                ws2812_encoder->state = 1;
            }
            if (session_state & RMT_ENCODING_MEM_FULL) {
                *ret_state = (rmt_encode_state_t)(session_state & (~RMT_ENCODING_COMPLETE));
                return encoded_symbols;
            }
            // Fall through
        case 1: // Send reset code
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &ws2812_encoder->reset_code,
                                                     sizeof(ws2812_encoder->reset_code), &session_state);
            if (session_state & RMT_ENCODING_COMPLETE) {
                ws2812_encoder->state = RMT_ENCODING_RESET;
                *ret_state = RMT_ENCODING_COMPLETE;
            }
            break;
    }
    
    return encoded_symbols;
}

static esp_err_t rmt_del_ws2812_encoder(rmt_encoder_t *encoder)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_del_encoder(ws2812_encoder->bytes_encoder);
    rmt_del_encoder(ws2812_encoder->copy_encoder);
    free(ws2812_encoder);
    return ESP_OK;
}

static esp_err_t rmt_ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_ws2812_encoder_t *ws2812_encoder = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_encoder_reset(ws2812_encoder->bytes_encoder);
    rmt_encoder_reset(ws2812_encoder->copy_encoder);
    ws2812_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t rmt_new_ws2812_encoder(rmt_encoder_handle_t *ret_encoder, uint32_t resolution)
{
    rmt_ws2812_encoder_t *ws2812_encoder = calloc(1, sizeof(rmt_ws2812_encoder_t));
    if (!ws2812_encoder) {
        return ESP_ERR_NO_MEM;
    }
    
    ws2812_encoder->base.encode = rmt_encode_ws2812;
    ws2812_encoder->base.del = rmt_del_ws2812_encoder;
    ws2812_encoder->base.reset = rmt_ws2812_encoder_reset;
    
    // Calculate ticks from nanoseconds
    uint32_t ticks_per_ns = resolution / 1000000;  // ticks per nanosecond * 1000
    
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = (WS2812_T0H_NS * ticks_per_ns) / 1000,
            .level1 = 0,
            .duration1 = (WS2812_T0L_NS * ticks_per_ns) / 1000,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = (WS2812_T1H_NS * ticks_per_ns) / 1000,
            .level1 = 0,
            .duration1 = (WS2812_T1L_NS * ticks_per_ns) / 1000,
        },
        .flags.msb_first = 1,
    };
    
    esp_err_t ret = rmt_new_bytes_encoder(&bytes_encoder_config, &ws2812_encoder->bytes_encoder);
    if (ret != ESP_OK) {
        free(ws2812_encoder);
        return ret;
    }
    
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &ws2812_encoder->copy_encoder);
    if (ret != ESP_OK) {
        rmt_del_encoder(ws2812_encoder->bytes_encoder);
        free(ws2812_encoder);
        return ret;
    }
    
    // Reset code: low for >50us
    ws2812_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = (WS2812_RESET_NS * ticks_per_ns) / 1000,
        .level1 = 0,
        .duration1 = (WS2812_RESET_NS * ticks_per_ns) / 1000,
    };
    
    *ret_encoder = &ws2812_encoder->base;
    return ESP_OK;
}

esp_err_t led_init(void)
{
    if (initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing LED on GPIO %d", PIN_STATUS_LED);
    
    // Configure RMT TX channel
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = PIN_STATUS_LED,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,  // 10 MHz = 100ns per tick
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    
    esp_err_t ret = rmt_new_tx_channel(&tx_config, &led_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create WS2812 encoder
    ret = rmt_new_ws2812_encoder(&led_encoder, tx_config.resolution_hz);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create WS2812 encoder: %s", esp_err_to_name(ret));
        rmt_del_channel(led_channel);
        return ret;
    }
    
    // Enable channel
    ret = rmt_enable(led_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        rmt_del_encoder(led_encoder);
        rmt_del_channel(led_channel);
        return ret;
    }
    
    initialized = true;
    
    // Turn off LED initially
    led_off();
    
    ESP_LOGI(TAG, "LED initialized");
    return ESP_OK;
}

void led_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!initialized) {
        return;
    }
    
    current_r = red;
    current_g = green;
    current_b = blue;
    
    // Standard WS2812 uses GRB order, but some variants use RGB
    // This LED appears to use RGB order
    uint8_t rgb[3] = {red, green, blue};
    
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    
    rmt_transmit(led_channel, led_encoder, rgb, sizeof(rgb), &tx_config);
    rmt_tx_wait_all_done(led_channel, portMAX_DELAY);
}

void led_off(void)
{
    led_set_color(0, 0, 0);
    locate_active = false;
}

void led_set_locate(bool enable)
{
    locate_active = enable;
    if (enable) {
        // Blue for locate
        led_set_color(0, 0, 255);
    } else {
        led_off();
    }
}

void led_flash_locate(uint32_t duration_ms)
{
    if (!initialized) {
        return;
    }
    
    locate_active = true;
    
    // Flash blue
    led_set_color(0, 0, 255);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    led_off();
    
    locate_active = false;
}

bool led_is_locate_active(void)
{
    return locate_active;
}
