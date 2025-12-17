// PWM-based buzzer using LEDC

#include "buzzer.h"
#include "pinout.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static bool initialized = false;
static bool buzzer_state = false;
static TaskHandle_t locate_task_handle = NULL;
static SemaphoreHandle_t locate_mutex = NULL;
static volatile bool locate_active = false;

// LEDC configuration
#define BUZZER_LEDC_TIMER      LEDC_TIMER_0
#define BUZZER_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_CHANNEL    LEDC_CHANNEL_0
#define BUZZER_LEDC_DUTY       4000   // duty in range 0-8191 for 13-bit
#define BUZZER_LEDC_DUTY_MAX   8191

// Default locate tone (Hz)
#define BUZZER_LOCATE_FREQ_HZ  2000

// Locate pattern: a small set of tones to alternate between
static const uint32_t locate_tones[] = {2000, 2500, 1800};
static const size_t locate_tone_count = sizeof(locate_tones) / sizeof(locate_tones[0]);
// Duration of each tone in ms
#define LOCATE_TONE_MS 120
// Silence between tones (ms)
#define LOCATE_SILENCE_MS 40

esp_err_t buzzer_init(void)
{
    if (initialized) return ESP_OK;

    if (locate_mutex == NULL) {
        locate_mutex = xSemaphoreCreateMutex();
    }

    // Configure timer
    ledc_timer_config_t timer_cfg = {
        .speed_mode = BUZZER_LEDC_MODE,
        .timer_num = BUZZER_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = BUZZER_LOCATE_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) return ret;

    // Configure channel
    ledc_channel_config_t ch_cfg = {
        .gpio_num = PIN_CHIRPER,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ret = ledc_channel_config(&ch_cfg);
    if (ret != ESP_OK) return ret;

    // Start with 0 duty (off)
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);

    initialized = true;
    buzzer_state = false;
    return ESP_OK;
}

esp_err_t buzzer_play_tone(uint32_t freq_hz)
{
    if (!initialized) {
        esp_err_t ret = buzzer_init();
        if (ret != ESP_OK) return ret;
    }

    if (freq_hz == 0) {
        buzzer_stop();
        return ESP_OK;
    }

    // Reconfigure timer frequency
    ledc_timer_config_t timer_cfg = {
        .speed_mode = BUZZER_LEDC_MODE,
        .timer_num = BUZZER_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_cfg);
    if (ret != ESP_OK) return ret;

    // Set duty (50% approx)
    uint32_t duty = (BUZZER_LEDC_DUTY_MAX / 2);
    ret = ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, duty);
    if (ret != ESP_OK) return ret;
    ret = ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    if (ret != ESP_OK) return ret;

    buzzer_state = true;
    return ESP_OK;
}

void buzzer_stop(void)
{
    if (!initialized) return;
    ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    buzzer_state = false;
}

static void locate_task(void *arg)
{
    size_t idx = 0;
    while (locate_active) {
        uint32_t freq = locate_tones[idx % locate_tone_count];
        buzzer_play_tone(freq);
        vTaskDelay(pdMS_TO_TICKS(LOCATE_TONE_MS));
        buzzer_stop();
        vTaskDelay(pdMS_TO_TICKS(LOCATE_SILENCE_MS));
        idx++;
    }

    // Ensure buzzer off when exiting
    buzzer_stop();
    locate_task_handle = NULL;
    vTaskDelete(NULL);
}

void buzzer_set_locate(bool enable)
{
    if (!initialized) {
        if (buzzer_init() != ESP_OK) return;
    }

    // Guard start/stop with mutex
    if (locate_mutex) xSemaphoreTake(locate_mutex, portMAX_DELAY);

    if (enable && !locate_active) {
        locate_active = true;
        // Start task if not already running
        BaseType_t ret = xTaskCreate(locate_task, "buzzer_locate", 2048, NULL, 3, &locate_task_handle);
        if (ret != pdPASS) {
            locate_active = false;
            if (locate_mutex) xSemaphoreGive(locate_mutex);
            return;
        }
    } else if (!enable && locate_active) {
        // Signal task to stop
        locate_active = false;
        // Wait briefly for task to clear handle
        int wait = 0;
        while (locate_task_handle != NULL && wait < 50) {
            vTaskDelay(pdMS_TO_TICKS(10));
            wait++;
        }
    }

    if (locate_mutex) xSemaphoreGive(locate_mutex);
}

bool buzzer_is_on(void)
{
    return buzzer_state;
}

