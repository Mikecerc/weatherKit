/**
 * @file buttons.c
 * @brief Interrupt-driven button handler with debouncing
 * 
 * Uses GPIO interrupts to detect button presses, then a task
 * for debouncing and long-press detection. More power efficient
 * than polling since the task only runs when buttons are active.
 */

#include "buttons.h"
#include "pinout.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "buttons";

// Timing configuration
#define DEBOUNCE_TIME_MS    50
#define LONG_PRESS_TIME_MS  2000
#define POLL_INTERVAL_MS    20      // Polling interval while buttons are active

// Button state tracking
typedef struct {
    gpio_num_t gpio;
    bool raw_state;             // Current GPIO state (after debounce)
    bool stable_state;          // Confirmed stable state
    int64_t press_start_us;     // When button was pressed (microseconds)
    bool long_press_fired;      // Long press event already sent
    bool processed;             // Button event already processed (for both-press)
    int64_t last_change_us;     // Last state change time for debounce
} button_state_t;

static button_state_t buttons[BUTTON_COUNT];
static button_callback_t event_callback = NULL;
static TaskHandle_t button_task_handle = NULL;
static SemaphoreHandle_t button_semaphore = NULL;

// Both-button tracking
static int64_t both_pressed_start_us = 0;
static bool both_long_press_fired = false;
static bool any_press_sent = false;

// Task state
static volatile bool buttons_active = false;

/**
 * @brief GPIO ISR handler - wakes the button task
 */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Wake the button task
    if (button_semaphore != NULL) {
        xSemaphoreGiveFromISR(button_semaphore, &xHigherPriorityTaskWoken);
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Read raw button state (active low - pressed = 0)
 */
static inline bool read_button_raw(button_id_t button)
{
    return gpio_get_level(buttons[button].gpio) == 0;
}

/**
 * @brief Update button state with debouncing
 * @return true if state changed
 */
static bool update_button_state(button_id_t button)
{
    button_state_t *btn = &buttons[button];
    bool current_raw = read_button_raw(button);
    int64_t now_us = esp_timer_get_time();
    
    // Check if raw state changed
    if (current_raw != btn->raw_state) {
        btn->raw_state = current_raw;
        btn->last_change_us = now_us;
        return false;  // Wait for debounce
    }
    
    // Check debounce time
    if ((now_us - btn->last_change_us) < (DEBOUNCE_TIME_MS * 1000)) {
        return false;  // Still debouncing
    }
    
    // State is stable - check if it changed from confirmed state
    if (btn->raw_state != btn->stable_state) {
        btn->stable_state = btn->raw_state;
        
        if (btn->stable_state) {
            // Button just pressed
            btn->press_start_us = now_us;
            btn->long_press_fired = false;
            btn->processed = false;
        }
        return true;  // State changed
    }
    
    return false;
}

/**
 * @brief Process individual button for long press and release events
 */
static void process_button_events(button_id_t button)
{
    button_state_t *btn = &buttons[button];
    int64_t now_us = esp_timer_get_time();
    
    if (btn->stable_state) {
        // Button is held - check for long press
        if (!btn->long_press_fired && !btn->processed) {
            int64_t held_ms = (now_us - btn->press_start_us) / 1000;
            if (held_ms >= LONG_PRESS_TIME_MS) {
                btn->long_press_fired = true;
                if (event_callback) {
                    button_event_t event = (button == BUTTON_LEFT) ? 
                        BUTTON_LEFT_LONG : BUTTON_RIGHT_LONG;
                    event_callback(event);
                }
            }
        }
    } else {
        // Button was just released
        if (!btn->long_press_fired && !btn->processed && event_callback) {
            // Short press
            button_event_t event = (button == BUTTON_LEFT) ? 
                BUTTON_LEFT_SHORT : BUTTON_RIGHT_SHORT;
            event_callback(event);
        }
        // Reset for next press
        btn->processed = false;
    }
}

/**
 * @brief Button processing task
 * Woken by ISR, processes button states and generates events
 */
static void button_task(void *pvParameters)
{
    while (1) {
        // Wait for button activity (ISR or timeout while active)
        TickType_t wait_time = buttons_active ? pdMS_TO_TICKS(POLL_INTERVAL_MS) : portMAX_DELAY;
        xSemaphoreTake(button_semaphore, wait_time);
        
        // Update all button states
        for (int i = 0; i < BUTTON_COUNT; i++) {
            update_button_state(i);
        }
        
        // Get current states
        bool left_pressed = buttons[BUTTON_LEFT].stable_state;
        bool right_pressed = buttons[BUTTON_RIGHT].stable_state;
        bool both_pressed = left_pressed && right_pressed;
        bool any_pressed = left_pressed || right_pressed;
        int64_t now_us = esp_timer_get_time();
        
        // Track if buttons are active (for polling interval vs infinite wait)
        buttons_active = any_pressed;
        
        // Send ANY_PRESS once when any button is first pressed
        if (any_pressed && !any_press_sent && event_callback) {
            event_callback(BUTTON_ANY_PRESS);
            any_press_sent = true;
        }
        
        // Reset ANY_PRESS tracking when all buttons released
        if (!any_pressed) {
            any_press_sent = false;
        }
        
        // Handle BOTH buttons pressed - takes priority
        if (both_pressed) {
            if (both_pressed_start_us == 0) {
                // Just started pressing both
                both_pressed_start_us = now_us;
                both_long_press_fired = false;
                // Suppress individual button events
                buttons[BUTTON_LEFT].processed = true;
                buttons[BUTTON_RIGHT].processed = true;
            } else if (!both_long_press_fired) {
                // Check for long press
                int64_t held_ms = (now_us - both_pressed_start_us) / 1000;
                if (held_ms >= LONG_PRESS_TIME_MS) {
                    both_long_press_fired = true;
                    buttons[BUTTON_LEFT].long_press_fired = true;
                    buttons[BUTTON_RIGHT].long_press_fired = true;
                    
                    if (event_callback) {
                        event_callback(BUTTON_BOTH_LONG);
                    }
                }
            }
        } else {
            // Not both pressed - reset tracking
            if (both_pressed_start_us != 0) {
                both_pressed_start_us = 0;
                both_long_press_fired = false;
            }
            
            // Process individual buttons
            for (int i = 0; i < BUTTON_COUNT; i++) {
                process_button_events(i);
            }
        }
    }
}

esp_err_t buttons_init(button_callback_t callback)
{
    ESP_LOGI(TAG, "Initializing interrupt-driven buttons...");
    
    event_callback = callback;
    
    // Create semaphore for ISR->task signaling
    button_semaphore = xSemaphoreCreateBinary();
    if (button_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return ESP_FAIL;
    }
    
    // Configure button GPIOs
    buttons[BUTTON_LEFT].gpio = PIN_BUTTON_LEFT;
    buttons[BUTTON_RIGHT].gpio = PIN_BUTTON_RIGHT;
    
    // Install GPIO ISR service
    gpio_install_isr_service(0);
    
    for (int i = 0; i < BUTTON_COUNT; i++) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << buttons[i].gpio),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,  // External pull-ups present
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_ANYEDGE      // Interrupt on both edges
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf));
        
        // Add ISR handler
        ESP_ERROR_CHECK(gpio_isr_handler_add(buttons[i].gpio, button_isr_handler, NULL));
        
        // Initialize state
        bool currently_pressed = read_button_raw(i);
        buttons[i].raw_state = currently_pressed;
        buttons[i].stable_state = currently_pressed;
        buttons[i].press_start_us = 0;
        buttons[i].long_press_fired = false;
        buttons[i].processed = false;
        buttons[i].last_change_us = esp_timer_get_time();
    }
    
    // Create button task
    BaseType_t ret = xTaskCreate(
        button_task,
        "button_task",
        3072,
        NULL,
        5,     // Medium priority
        &button_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task");
        vSemaphoreDelete(button_semaphore);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Buttons initialized (LEFT=GPIO%d, RIGHT=GPIO%d) [interrupt-driven]", 
             PIN_BUTTON_LEFT, PIN_BUTTON_RIGHT);
    return ESP_OK;
}

bool buttons_is_pressed(button_id_t button)
{
    if (button >= BUTTON_COUNT) return false;
    return buttons[button].stable_state;
}
