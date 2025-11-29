#include "buttons.h"
#include "pinout.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "buttons";

// Timing configuration (in ms)
#define DEBOUNCE_TIME_MS    50
#define LONG_PRESS_TIME_MS  2000
#define POLL_INTERVAL_MS    20

// Button state tracking
typedef struct {
    gpio_num_t gpio;
    bool last_state;
    bool current_state;
    bool processed;
    TickType_t press_start_tick;
    bool long_press_fired;
} button_state_t;

static button_state_t buttons[BUTTON_COUNT];
static button_callback_t event_callback = NULL;
static TaskHandle_t button_task_handle = NULL;

/**
 * @brief Read button state (active low - pressed = 0)
 */
static bool read_button(button_id_t button)
{
    return gpio_get_level(buttons[button].gpio) == 0;
}

/**
 * @brief Process button state and generate events
 */
static void process_button(button_id_t button)
{
    button_state_t *btn = &buttons[button];
    bool pressed = read_button(button);
    TickType_t now = xTaskGetTickCount();
    
    // Debounce
    if (pressed != btn->last_state) {
        btn->last_state = pressed;
        return;  // Wait for next poll to confirm
    }
    
    // State confirmed
    if (pressed != btn->current_state) {
        btn->current_state = pressed;
        
        if (pressed) {
            // Button just pressed
            btn->press_start_tick = now;
            btn->long_press_fired = false;
            btn->processed = false;
        } else {
            // Button just released
            if (!btn->long_press_fired && !btn->processed && event_callback) {
                // Short press - send combined event
                button_event_t event = (button == BUTTON_LEFT) ? 
                    BUTTON_LEFT_SHORT : BUTTON_RIGHT_SHORT;
                event_callback(event);
            }
        }
    }
    
    // Check for long press while held
    if (pressed && !btn->long_press_fired) {
        TickType_t held_ms = (now - btn->press_start_tick) * portTICK_PERIOD_MS;
        if (held_ms >= LONG_PRESS_TIME_MS) {
            btn->long_press_fired = true;
            if (event_callback) {
                button_event_t event = (button == BUTTON_LEFT) ? 
                    BUTTON_LEFT_LONG : BUTTON_RIGHT_LONG;
                event_callback(event);
            }
        }
    }
}

/**
 * @brief Button polling task
 */
static void button_task(void *pvParameters)
{
    while (1) {
        for (int i = 0; i < BUTTON_COUNT; i++) {
            process_button(i);
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

esp_err_t buttons_init(button_callback_t callback)
{
    ESP_LOGI(TAG, "Initializing buttons...");
    
    event_callback = callback;
    
    // Configure button GPIOs
    buttons[BUTTON_LEFT].gpio = PIN_BUTTON_LEFT;
    buttons[BUTTON_RIGHT].gpio = PIN_BUTTON_RIGHT;
    
    for (int i = 0; i < BUTTON_COUNT; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << buttons[i].gpio),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,  // External pull-ups present
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        
        // Initialize state to match current button state (unpressed = high = false for "pressed")
        bool currently_pressed = (gpio_get_level(buttons[i].gpio) == 0);
        buttons[i].last_state = currently_pressed;
        buttons[i].current_state = currently_pressed;
        buttons[i].processed = false;
        buttons[i].press_start_tick = 0;
        buttons[i].long_press_fired = false;
    }
    
    // Create button task - minimal stack since no logging in loop
    BaseType_t ret = xTaskCreate(
        button_task,
        "button_task",
        2048,  // Sufficient for simple polling
        NULL,
        5,     // Medium priority
        &button_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Buttons initialized (LEFT=GPIO%d, RIGHT=GPIO%d)", 
             PIN_BUTTON_LEFT, PIN_BUTTON_RIGHT);
    return ESP_OK;
}

bool buttons_is_pressed(button_id_t button)
{
    if (button >= BUTTON_COUNT) return false;
    return buttons[button].current_state;
}
