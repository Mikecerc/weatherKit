/**
 * @file lora.c
 * @brief LoRa driver for SX1278/RA-02 module (Sensor Package Version)
 * 
 * Based on Inteform/esp32-lora-library
 * https://github.com/Inteform/esp32-lora-library
 * 
 * Modified for WeatherKit Sensor Package:
 * - Uses pinout.h for configuration
 * - No UI dependencies (standalone sensor)
 * - Adds status tracking
 * - Interrupt-driven RX using DIO0
 */

#include "lora.h"
#include "pinout.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include "esp_timer.h"

static const char *TAG = "lora";

// =============================================================================
// SX1278 Register Definitions
// =============================================================================
#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_LNA                        0x0c
#define REG_FIFO_ADDR_PTR              0x0d
#define REG_FIFO_TX_BASE_ADDR          0x0e
#define REG_FIFO_RX_BASE_ADDR          0x0f
#define REG_FIFO_RX_CURRENT_ADDR       0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_PKT_SNR_VALUE              0x19
#define REG_PKT_RSSI_VALUE             0x1a
#define REG_MODEM_CONFIG_1             0x1d
#define REG_MODEM_CONFIG_2             0x1e
#define REG_PREAMBLE_MSB               0x20
#define REG_PREAMBLE_LSB               0x21
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_RSSI_WIDEBAND              0x2c
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_DETECTION_THRESHOLD        0x37
#define REG_SYNC_WORD                  0x39
#define REG_DIO_MAPPING_1              0x40
#define REG_VERSION                    0x42

// Transceiver modes
#define MODE_LONG_RANGE_MODE           0x80
#define MODE_SLEEP                     0x00
#define MODE_STDBY                     0x01
#define MODE_TX                        0x03
#define MODE_RX_CONTINUOUS             0x05
#define MODE_RX_SINGLE                 0x06

// PA configuration
#define PA_BOOST                       0x80

// IRQ masks
#define IRQ_TX_DONE_MASK               0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK     0x20
#define IRQ_RX_DONE_MASK               0x40

// DIO0 mapping values (bits 7:6 of REG_DIO_MAPPING_1)
#define DIO0_RX_DONE                   0x00  // DIO0 = RxDone

#define TIMEOUT_RESET                  100

// =============================================================================
// Static Variables
// =============================================================================
static spi_device_handle_t spi_handle = NULL;
static bool initialized = false;
static int implicit_header = 0;
static long current_frequency = 433000000;  // Default 433 MHz

// Status tracking
static lora_status_t status = {0};

// Sequence number for outgoing packets
static uint8_t tx_sequence = 0;

// Interrupt-driven RX
static SemaphoreHandle_t rx_semaphore = NULL;
static volatile bool rx_done_flag = false;

// Callbacks for received data (sensor package specific)
static lora_config_cb_t config_callback = NULL;
static lora_weather_ack_cb_t weather_ack_callback = NULL;

// High power mode state
static bool high_power_enabled = false;

// Adaptive power state
static bool adaptive_power_enabled = false;
static int8_t recent_rssi = RSSI_POOR;  // Start with "excellent" to keep power low until we have real RSSI data
static uint8_t current_tx_power = LORA_TX_POWER_LOW;

// =============================================================================
// Interrupt Handler
// =============================================================================

/**
 * @brief DIO0 interrupt handler - called on RX done or TX done
 */
static void IRAM_ATTR lora_dio0_isr(void *arg)
{
    (void)arg;
    rx_done_flag = true;
    
    // Wake up any task waiting for RX
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (rx_semaphore != NULL) {
        xSemaphoreGiveFromISR(rx_semaphore, &xHigherPriorityTaskWoken);
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// =============================================================================
// Low-level SPI Functions
// =============================================================================

/**
 * @brief Write a value to a register
 */
static void write_reg(int reg, int val)
{
    uint8_t out[2] = { 0x80 | reg, val };
    uint8_t in[2];

    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * sizeof(out),
        .tx_buffer = out,
        .rx_buffer = in
    };

    gpio_set_level(PIN_LORA_NSS, 0);
    spi_device_transmit(spi_handle, &t);
    gpio_set_level(PIN_LORA_NSS, 1);
}

/**
 * @brief Read a value from a register
 */
static int read_reg(int reg)
{
    uint8_t out[2] = { reg, 0xff };
    uint8_t in[2] = { 0, 0 };

    spi_transaction_t t = {
        .flags = 0,
        .length = 8 * sizeof(out),
        .tx_buffer = out,
        .rx_buffer = in
    };

    gpio_set_level(PIN_LORA_NSS, 0);
    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    gpio_set_level(PIN_LORA_NSS, 1);
    
    // Debug: log SPI transaction for version register
    if (reg == 0x42) {
        ESP_LOGI(TAG, "SPI DEBUG: reg=0x%02X, tx=[0x%02X,0x%02X], rx=[0x%02X,0x%02X], ret=%s",
                 reg, out[0], out[1], in[0], in[1], esp_err_to_name(ret));
    }
    
    return in[1];
}

/**
 * @brief Perform hardware reset
 */
static void hardware_reset(void)
{
    ESP_LOGI(TAG, "Performing hardware reset (RST pin = GPIO %d)", PIN_LORA_RST);
    gpio_set_level(PIN_LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));  // Hold reset longer
    gpio_set_level(PIN_LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));  // Wait longer for chip to stabilize
    ESP_LOGI(TAG, "Hardware reset complete");
}

// =============================================================================
// Public API
// =============================================================================

esp_err_t lora_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing LoRa module...");
    ESP_LOGI(TAG, "Pin config: SCK=%d, MISO=%d, MOSI=%d, NSS=%d, RST=%d, DIO0=%d",
             PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS, PIN_LORA_RST, PIN_LORA_DIO0);

    // Configure GPIO pins
    gpio_reset_pin(PIN_LORA_RST);
    gpio_reset_pin(PIN_LORA_NSS);
    
    gpio_set_direction(PIN_LORA_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_LORA_NSS, GPIO_MODE_OUTPUT);
    
    gpio_set_level(PIN_LORA_RST, 1);
    gpio_set_level(PIN_LORA_NSS, 1);

    // Configure DIO0 as input with interrupt
    gpio_config_t dio0_config = {
        .pin_bit_mask = (1ULL << PIN_LORA_DIO0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLDOWN_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE  // Interrupt on rising edge
    };
    gpio_config(&dio0_config);

    // Create semaphore for RX notification
    if (rx_semaphore == NULL) {
        rx_semaphore = xSemaphoreCreateBinary();
    }

    // Install GPIO ISR service and add handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_LORA_DIO0, lora_dio0_isr, NULL);

    // Initialize SPI bus
    spi_bus_config_t bus_config = {
        .miso_io_num = PIN_LORA_MISO,
        .mosi_io_num = PIN_LORA_MOSI,
        .sclk_io_num = PIN_LORA_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256
    };

    ret = spi_bus_initialize(LORA_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Add SPI device
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = LORA_SPI_FREQ_HZ,
        .mode = 0,
        .spics_io_num = -1,  // Manual CS control
        .queue_size = 1,
        .flags = 0,
        .pre_cb = NULL
    };

    ret = spi_bus_add_device(LORA_SPI_HOST, &dev_config, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        spi_bus_free(LORA_SPI_HOST);
        return ret;
    }

    // Hardware reset
    hardware_reset();
    
    // Read version register to verify chip is present
    uint8_t version = read_reg(REG_VERSION);

    if (version != 0x12) {
        ESP_LOGE(TAG, "SX1278 not found (version=0x%02X, expected 0x12)", version);
        spi_bus_remove_device(spi_handle);
        spi_bus_free(LORA_SPI_HOST);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "SX1278 detected (version=0x%02X)", version);

    // Default configuration
    lora_sleep();
    write_reg(REG_FIFO_RX_BASE_ADDR, 0);
    write_reg(REG_FIFO_TX_BASE_ADDR, 0);
    write_reg(REG_LNA, read_reg(REG_LNA) | 0x03);  // Max LNA gain
    write_reg(REG_MODEM_CONFIG_3, 0x04);           // Auto AGC

    // Configure DIO0 to trigger on RxDone (default mode)
    // DIO0 mapping: 00 = RxDone, 01 = TxDone
    write_reg(REG_DIO_MAPPING_1, DIO0_RX_DONE);

    // Set initial TX power (low power for sensor package)
    lora_set_tx_power(LORA_TX_POWER_LOW);
    current_tx_power = LORA_TX_POWER_LOW;

    // Default settings
    lora_set_frequency(433000000);  // 915 MHz (US ISM band)
    lora_set_spreading_factor(7);   // SF7 (fastest)
    lora_set_bandwidth(125000);     // 125 kHz
    lora_set_coding_rate(5);        // 4/5
    lora_enable_crc();
    lora_set_sync_word(LORA_SYNC_WORD);  // Use our network sync word

    lora_idle();
    
        // Verify chip is responding correctly with a write-read test
    write_reg(REG_FIFO_ADDR_PTR, 0xAA);
    write_reg(REG_FIFO_TX_BASE_ADDR, 0x55);
    vTaskDelay(pdMS_TO_TICKS(1));
    
    uint8_t reg1_read = read_reg(REG_FIFO_ADDR_PTR);
    uint8_t reg2_read = read_reg(REG_FIFO_TX_BASE_ADDR);
    
    if (reg1_read != 0xAA || reg2_read != 0x55) {
        ESP_LOGE(TAG, "Register write-read test FAILED");
        spi_bus_remove_device(spi_handle);
        spi_bus_free(LORA_SPI_HOST);
        return ESP_ERR_INVALID_RESPONSE;
    } else {
        ESP_LOGI(TAG, "Register write-read test PASSED");
    }
    
    // Reset registers to 0
    write_reg(REG_FIFO_ADDR_PTR, 0);
    write_reg(REG_FIFO_TX_BASE_ADDR, 0);


    // Reset status
    memset(&status, 0, sizeof(status));
    status.initialized = true;
    status.tx_power = current_tx_power;
    initialized = true;

    ESP_LOGI(TAG, "LoRa module initialized successfully");
    return ESP_OK;
}

bool lora_is_initialized(void)
{
    return initialized;
}

void lora_idle(void)
{
    write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);

     // Wait for mode transition to complete
    int retries = 0;
    while ((read_reg(REG_OP_MODE) & 0x07) != MODE_STDBY && retries < 10) {
        vTaskDelay(pdMS_TO_TICKS(1));
        retries++;
    }
}

void lora_sleep(void)
{
    write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void lora_receive(void)
{
    if (!initialized) return;
    
    // Check if already in RX continuous mode - don't disrupt
    uint8_t mode = read_reg(REG_OP_MODE);
    if ((mode & 0x87) == (MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS)) {
        // Already in RX mode, don't touch anything
        return;
    }
    
    // Configure DIO0 for RxDone interrupt
    write_reg(REG_DIO_MAPPING_1, DIO0_RX_DONE);
    
    // Clear any pending flags only when first entering RX
    rx_done_flag = false;
    write_reg(REG_IRQ_FLAGS, 0xFF);
    
    // Enter continuous receive mode
    write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
    
    ESP_LOGD(TAG, "Entered RX mode (was 0x%02X)", mode);
}

void lora_set_frequency(long frequency)
{
    current_frequency = frequency;

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

    write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void lora_set_tx_power(int level)
{
     // PA config should be changed in standby mode
    lora_idle();
    
    uint8_t pa_config;
    int reported_power;
    
    // Use RFO output (not PA_BOOST) for lower power levels
    // RFO range: -4 to +14 dBm (MaxPower=0-7, OutputPower=0-15)
    // PA_BOOST range: +2 to +17 dBm
    
    if (level <= 0) {
        // Use RFO at minimum power for testing without antenna
        // PA_CONFIG: PA_SELECT=0 (RFO), MaxPower=0, OutputPower=0 = -4 dBm
        pa_config = 0x00;
        reported_power = -4;
    } else if (level < 2) {
        // RFO low power
        pa_config = 0x70 | level;  // MaxPower=7, OutputPower=level
        reported_power = level;
    } else {
        // PA_BOOST for +2 to +17 dBm
        if (level > 17) level = 17;
        pa_config = PA_BOOST | (level - 2);
        reported_power = level;
    }
    
    write_reg(REG_PA_CONFIG, pa_config);
    status.tx_power = reported_power;
    
    // Small delay to let PA stabilize
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // Read back and verify
    uint8_t readback = read_reg(REG_PA_CONFIG);
    if (readback != pa_config) {
        ESP_LOGE(TAG, "TX power MISMATCH! Wrote 0x%02X, read 0x%02X", pa_config, readback);
    } else {
        ESP_LOGI(TAG, "TX power set to %d dBm (PA_CONFIG=0x%02X verified)", reported_power, readback);
    }
}

void lora_set_spreading_factor(int sf)
{
    if (sf < 6) sf = 6;
    else if (sf > 12) sf = 12;

    if (sf == 6) {
        write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
        write_reg(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
        write_reg(REG_DETECTION_THRESHOLD, 0x0a);
    }

    write_reg(REG_MODEM_CONFIG_2, (read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void lora_set_bandwidth(long sbw)
{
    int bw;

    if (sbw <= 7800) bw = 0;
    else if (sbw <= 10400) bw = 1;
    else if (sbw <= 15600) bw = 2;
    else if (sbw <= 20800) bw = 3;
    else if (sbw <= 31250) bw = 4;
    else if (sbw <= 41700) bw = 5;
    else if (sbw <= 62500) bw = 6;
    else if (sbw <= 125000) bw = 7;
    else if (sbw <= 250000) bw = 8;
    else bw = 9;

    write_reg(REG_MODEM_CONFIG_1, (read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void lora_set_coding_rate(int denominator)
{
    if (denominator < 5) denominator = 5;
    else if (denominator > 8) denominator = 8;

    int cr = denominator - 4;
    write_reg(REG_MODEM_CONFIG_1, (read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void lora_enable_crc(void)
{
    write_reg(REG_MODEM_CONFIG_2, read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

void lora_disable_crc(void)
{
    write_reg(REG_MODEM_CONFIG_2, read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

void lora_set_sync_word(int sw)
{
    write_reg(REG_SYNC_WORD, sw);
}

esp_err_t lora_send_packet(const uint8_t *buf, int size)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (size > 255) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Put chip in standby mode before TX
    lora_idle();
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure DIO0 for TX Done interrupt
    //write_reg(REG_DIO_MAPPING_1, DIO0_TX_DONE);
    
    // Clear any pending IRQ flags and reset interrupt flag
    write_reg(REG_IRQ_FLAGS, 0xFF);
    rx_done_flag = false;
    
    write_reg(REG_FIFO_ADDR_PTR, 0);
    write_reg(REG_FIFO_TX_BASE_ADDR, 0);

    // Write data to FIFO
    for (int i = 0; i < size; i++) {
        write_reg(REG_FIFO, buf[i]);
    }
    write_reg(REG_PAYLOAD_LENGTH, size);


    // Verify FIFO pointer advanced correctly
    uint8_t fifo_ptr = read_reg(REG_FIFO_ADDR_PTR);
    if (fifo_ptr != size) {
        ESP_LOGE(TAG, "FIFO write error: ptr=%d, expected=%d", fifo_ptr, size);
    }

    // Verify we're in standby before TX
    uint8_t mode_before = read_reg(REG_OP_MODE);
    uint8_t irq_before = read_reg(REG_IRQ_FLAGS);
    ESP_LOGD(TAG, "TX %d bytes: OpMode=0x%02X, IRQ=0x%02X, FIFO=%d", size, mode_before, irq_before, fifo_ptr);


    // Start transmission
    write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    
    /**
     // Wait for TX done using hardware DIO0 pin
    int timeout = 0;
    while (!rx_done_flag && gpio_get_level(PIN_LORA_DIO0) == 0) {
        vTaskDelay(pdMS_TO_TICKS(2));
        if (++timeout > 500) {  // 1 second timeout
            ESP_LOGE(TAG, "TX timeout");
            lora_idle();
            write_reg(REG_DIO_MAPPING_1, DIO0_RX_DONE);
            return ESP_ERR_TIMEOUT;
        }
    }

    // Verify TX completed
    uint8_t irq_flags = read_reg(REG_IRQ_FLAGS);
    if ((irq_flags & IRQ_TX_DONE_MASK) == 0) {
        ESP_LOGE(TAG, "TX failed - IRQ_TX_DONE not set");
        lora_idle();
        write_reg(REG_DIO_MAPPING_1, DIO0_RX_DONE);
        return ESP_ERR_INVALID_STATE;
    }
    */
    /** */
    // Wait for TX done
    int64_t start_time = esp_timer_get_time();
    int timeout = 0;
    while ((read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout++;
        
        if (timeout > 200) {  // 2 second timeout (200 * 10ms)
            uint8_t irq_flags = read_reg(REG_IRQ_FLAGS);
            uint8_t op_mode = read_reg(REG_OP_MODE);
            int elapsed_ms = (int)((esp_timer_get_time() - start_time) / 1000);
            ESP_LOGE(TAG, "TX timeout after %dms - IRQ: 0x%02X, OpMode: 0x%02X", 
                     elapsed_ms, irq_flags, op_mode);
            lora_idle();
            return ESP_ERR_TIMEOUT;
        }
    }
    
    int elapsed_ms = (int)((esp_timer_get_time() - start_time) / 1000);
    ESP_LOGD(TAG, "TX done in %dms (%d bytes)", elapsed_ms, size);

    // Clear IRQ
    write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    
    // Return to standby and restore DIO0 for RX
    lora_idle();
    write_reg(REG_DIO_MAPPING_1, DIO0_RX_DONE);
    rx_done_flag = false;
    
    status.packets_sent++;

    return ESP_OK;
}

bool lora_received(void)
{
    if (!initialized) return false;
    
    // Check interrupt flag first (faster), then verify with register
    if (rx_done_flag) {
        return true;
    }
    return (read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) != 0;
}

bool lora_wait_receive(uint32_t timeout_ms)
{
    if (!initialized || rx_semaphore == NULL) return false;
    
    // Already have a packet?
    if (rx_done_flag || (read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK)) {
        return true;
    }
    
    // Wait for interrupt with timeout
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(rx_semaphore, ticks) == pdTRUE) {
        return true;
    }
    
    return false;
}

int lora_receive_packet(uint8_t *buf, int size)
{
    if (!initialized) return 0;

    int irq = read_reg(REG_IRQ_FLAGS);
    write_reg(REG_IRQ_FLAGS, irq);  // Clear flags
    rx_done_flag = false;           // Clear interrupt flag

    if ((irq & IRQ_RX_DONE_MASK) == 0) return 0;
    
    ESP_LOGI(TAG, "lora_receive_packet: IRQ flags = 0x%02X", irq);
    
    if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK) {
        status.crc_errors++;
        ESP_LOGW(TAG, "CRC error");
        return 0;
    }

    // Get packet length
    int len;
    if (implicit_header) {
        len = read_reg(REG_PAYLOAD_LENGTH);
    } else {
        len = read_reg(REG_RX_NB_BYTES);
    }

    // Read packet from FIFO
    lora_idle();
    write_reg(REG_FIFO_ADDR_PTR, read_reg(REG_FIFO_RX_CURRENT_ADDR));
    
    if (len > size) len = size;
    for (int i = 0; i < len; i++) {
        buf[i] = read_reg(REG_FIFO);
    }

    // Store RSSI/SNR
    status.last_rssi = lora_packet_rssi();
    status.last_snr = lora_packet_snr();
    status.packets_received++;

    ESP_LOGD(TAG, "Packet received (%d bytes, RSSI=%d, SNR=%.1f)", 
             len, status.last_rssi, status.last_snr);

    return len;
}

int lora_packet_rssi(void)
{
    return read_reg(REG_PKT_RSSI_VALUE) - (current_frequency < 868000000 ? 164 : 157);
}

float lora_packet_snr(void)
{
    return ((int8_t)read_reg(REG_PKT_SNR_VALUE)) * 0.25;
}

void lora_get_status(lora_status_t *out_status)
{
    if (out_status) {
        memcpy(out_status, &status, sizeof(lora_status_t));
    }
}

void lora_close(void)
{
    if (initialized) {
        lora_sleep();
        
        // Remove interrupt handler
        gpio_isr_handler_remove(PIN_LORA_DIO0);
        
        // Clean up SPI
        spi_bus_remove_device(spi_handle);
        spi_bus_free(LORA_SPI_HOST);
        
        // Clean up semaphore
        if (rx_semaphore != NULL) {
            vSemaphoreDelete(rx_semaphore);
            rx_semaphore = NULL;
        }
        
        initialized = false;
        status.initialized = false;
        ESP_LOGI(TAG, "LoRa module closed");
    }
}

void lora_dump_registers(void)
{
    printf("LoRa Registers:\n");
    printf("   00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
    for (int i = 0; i < 0x80; i++) {
        if ((i & 0x0f) == 0) printf("%02X:", i);
        printf(" %02X", read_reg(i));
        if ((i & 0x0f) == 0x0f) printf("\n");
    }
}

void lora_run_diagnostics(void)
{
    ESP_LOGI(TAG, "========== LoRa Diagnostics ==========");
    
    if (!initialized) {
        ESP_LOGE(TAG, "LoRa not initialized!");
        return;
    }
    
    // Check chip version
    uint8_t version = read_reg(REG_VERSION);
    ESP_LOGI(TAG, "Chip version: 0x%02X %s", version, 
             (version == 0x12) ? "(OK)" : "(UNEXPECTED!)");
    
    // Check operating mode
    uint8_t op_mode = read_reg(REG_OP_MODE);
    const char *mode_str = "Unknown";
    uint8_t mode = op_mode & 0x07;
    switch (mode) {
        case MODE_SLEEP:          mode_str = "Sleep"; break;
        case MODE_STDBY:          mode_str = "Standby"; break;
        case MODE_TX:             mode_str = "TX"; break;
        case MODE_RX_CONTINUOUS:  mode_str = "RX Continuous"; break;
        case MODE_RX_SINGLE:      mode_str = "RX Single"; break;
    }
    ESP_LOGI(TAG, "OpMode: 0x%02X (%s)", op_mode, mode_str);
    
    // Read frequency configuration
    uint8_t freq_msb = read_reg(REG_FRF_MSB);
    uint8_t freq_mid = read_reg(REG_FRF_MID);
    uint8_t freq_lsb = read_reg(REG_FRF_LSB);
    uint32_t freq_raw = ((uint32_t)freq_msb << 16) | ((uint32_t)freq_mid << 8) | freq_lsb;
    float freq_mhz = (float)freq_raw * 32.0f / (float)(1 << 19);
    ESP_LOGI(TAG, "Frequency: %.2f MHz", freq_mhz);
    
    // Check PA configuration
    uint8_t pa_config = read_reg(REG_PA_CONFIG);
    int tx_power = (pa_config & 0x0F) + 2;
    ESP_LOGI(TAG, "TX Power: %d dBm", tx_power);
    
    // Check modem configuration
    uint8_t config1 = read_reg(REG_MODEM_CONFIG_1);
    uint8_t config2 = read_reg(REG_MODEM_CONFIG_2);
    int sf = (config2 >> 4) & 0x0F;
    int bw = (config1 >> 4) & 0x0F;
    const char *bw_str[] = {"7.8", "10.4", "15.6", "20.8", "31.25", "41.7", "62.5", "125", "250", "500"};
    ESP_LOGI(TAG, "Modem: SF%d, BW=%s kHz, CRC=%s", 
             sf, (bw < 10) ? bw_str[bw] : "???",
             (config2 & 0x04) ? "On" : "Off");
    
    // Test noise floor
    lora_receive();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    int rssi_sum = 0;
    for (int i = 0; i < 5; i++) {
        int raw_rssi = read_reg(REG_RSSI_WIDEBAND);
        rssi_sum += raw_rssi - 157;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    int rssi_avg = rssi_sum / 5;
    ESP_LOGI(TAG, "Noise floor: %d dBm", rssi_avg);
    
    lora_idle();
    
    ESP_LOGI(TAG, "========== Diagnostics Complete ==========");
}

// =============================================================================
// Secure Messaging
// =============================================================================

/**
 * @brief CRC16-CCITT calculation
 */
uint16_t lora_crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

esp_err_t lora_send_secure(uint8_t dest_id, uint8_t msg_type,
                           const uint8_t *payload, int payload_len)
{
    if (!initialized) {
        ESP_LOGE(TAG, "lora_send_secure: Not initialized!");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (payload_len > LORA_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "lora_send_secure: Payload too large (%d > %d)", payload_len, LORA_MAX_PAYLOAD);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Build packet buffer
    uint8_t packet[256];
    lora_header_t *header = (lora_header_t *)packet;
    
    // Fill header - sensor package identifies as REMOTE
    header->device_id = LORA_DEVICE_ID_REMOTE;
    header->dest_id = dest_id;
    header->msg_type = msg_type;
    header->sequence = tx_sequence++;
    
    // Copy payload
    if (payload_len > 0 && payload != NULL) {
        memcpy(packet + LORA_HEADER_SIZE, payload, payload_len);
    }
    
    // Calculate checksum over header (minus checksum field) + payload
    // Temporarily set checksum to 0 for calculation
    header->checksum = 0;
    uint16_t crc = lora_crc16(packet, LORA_HEADER_SIZE + payload_len);
    header->checksum = crc;
    
    int total_len = LORA_HEADER_SIZE + payload_len;
    
    esp_err_t ret = lora_send_packet(packet, total_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TX failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

int lora_receive_secure(uint8_t *src_id, uint8_t *msg_type,
                        uint8_t *payload, int max_len)
{
    if (!initialized) {
        return 0;
    }
    
    // Receive raw packet
    uint8_t packet[256];
    int len = lora_receive_packet(packet, sizeof(packet));
    
    if (len < LORA_HEADER_SIZE) {
        // No packet or too short
        return 0;
    }
    
    lora_header_t *header = (lora_header_t *)packet;
    int payload_len = len - LORA_HEADER_SIZE;
    
    // Check destination (accept if addressed to us or broadcast)
    // Sensor package uses REMOTE device ID
    if (header->dest_id != LORA_DEVICE_ID_REMOTE && header->dest_id != 0xFF) {
        ESP_LOGW(TAG, "RX rejected: wrong dest (0x%02X, we are 0x%02X)", 
                 header->dest_id, LORA_DEVICE_ID_REMOTE);
        status.packets_rejected++;
        return 0;
    }
    
    // Verify checksum
    uint16_t received_crc = header->checksum;
    header->checksum = 0;  // Zero for calculation
    uint16_t calculated_crc = lora_crc16(packet, len);
    
    if (received_crc != calculated_crc) {
        ESP_LOGW(TAG, "RX rejected: bad checksum (got 0x%04X, expected 0x%04X)",
                 received_crc, calculated_crc);
        status.packets_rejected++;
        return 0;
    }
    
    // Packet validated!
    status.last_src_id = header->device_id;
    
    ESP_LOGI(TAG, "RX secure: src=0x%02X dest=0x%02X type=0x%02X seq=%d len=%d RSSI=%d",
             header->device_id, header->dest_id, header->msg_type, header->sequence, 
             payload_len, status.last_rssi);
    
    // Return info to caller
    if (src_id) *src_id = header->device_id;
    if (msg_type) *msg_type = header->msg_type;
    
    // Copy payload
    if (payload_len > 0 && payload != NULL) {
        int copy_len = (payload_len > max_len) ? max_len : payload_len;
        memcpy(payload, packet + LORA_HEADER_SIZE, copy_len);
        return copy_len;
    }
    
    return payload_len;
}

// =============================================================================
// High-Level Protocol API (Sensor Package)
// =============================================================================

void lora_set_config_callback(lora_config_cb_t callback)
{
    config_callback = callback;
}

void lora_set_weather_ack_callback(lora_weather_ack_cb_t callback)
{
    weather_ack_callback = callback;
}

void lora_set_adaptive_power(bool enable)
{
    adaptive_power_enabled = enable;
}

void lora_set_high_power(bool enable)
{
    high_power_enabled = enable;
}

uint8_t lora_get_suggested_power(void)
{
    // Based on RSSI thresholds from lora_protocol.h
    if (recent_rssi >= RSSI_EXCELLENT) {
        return TX_POWER_MIN;   // Minimum power for excellent link
    } else if (recent_rssi >= RSSI_GOOD) {
        return TX_POWER_LOW;   // Low power for good link
    } else if (recent_rssi >= RSSI_FAIR) {
        return TX_POWER_MED;   // Medium power for fair link
    } else if (recent_rssi >= RSSI_POOR) {
        return TX_POWER_HIGH;  // High power for poor link
    } else {
        return TX_POWER_MAX;   // Maximum power for critical/no link
    }
}

uint8_t lora_get_current_power(void)
{
    return current_tx_power;
}

int8_t lora_get_last_rssi(void)
{
    return status.last_rssi;
}

/**
 * @brief Apply adaptive power if enabled
 */
static void apply_adaptive_power(void)
{
    if (!adaptive_power_enabled) return;
    
    uint8_t suggested = lora_get_suggested_power();
    if (suggested != current_tx_power) {
        current_tx_power = suggested;
        lora_set_tx_power(current_tx_power);
        ESP_LOGD(TAG, "Adaptive power: RSSI=%d -> TX=%d dBm", recent_rssi, current_tx_power);
    }
}

esp_err_t lora_send_weather(const weather_payload_t *weather)
{
    if (!weather) return ESP_ERR_INVALID_ARG;
    
    apply_adaptive_power();
    
    //quickly check the dbm level and the paboost level by reading the reg directly
                    uint8_t pa_config = read_reg(0x09);
                    int8_t paboost = (pa_config & 0x80) ? 1 : 0;
                    int8_t output_power = pa_config & 0x0F;  // This is what you're missing

                    int8_t tx_dbm = paboost ? (2 + output_power) : (-4 + output_power);
                    ESP_LOGI(TAG, "power levels: PA_BOOST=%d, OutputPower=%d, TX Power=%d dBm", paboost, output_power, tx_dbm);

    return lora_send_secure(LORA_DEVICE_ID_BASE, MSG_TYPE_WEATHER_DATA,
                            (const uint8_t *)weather, sizeof(weather_payload_t));
}

esp_err_t lora_send_sensor_status(const status_payload_t *status_data)
{
    if (!status_data) return ESP_ERR_INVALID_ARG;
    
    apply_adaptive_power();
    
    return lora_send_secure(LORA_DEVICE_ID_BASE, MSG_TYPE_SENSOR_STATUS,
                            (const uint8_t *)status_data, sizeof(status_payload_t));
}

esp_err_t lora_send_config_ack(uint8_t config_seq, uint8_t applied_flags)
{
    config_ack_payload_t ack = {
        .acked_sequence = config_seq,
        .sensor_rssi = status.last_rssi,
        .sensor_tx_power = current_tx_power,
        .applied_flags = applied_flags
    };
    
    ESP_LOGI(TAG, "Sending CONFIG_ACK for seq=%d, flags=0x%02X", config_seq, applied_flags);
    return lora_send_secure(LORA_DEVICE_ID_BASE, MSG_TYPE_CONFIG_ACK,
                            (const uint8_t *)&ack, sizeof(config_ack_payload_t));
}

esp_err_t lora_send_weather_ack_ack(uint8_t acked_seq, uint32_t lightning_total)
{
    weather_ack_ack_payload_t ack_ack = {
        .acked_sequence = acked_seq,
        .lightning_total = lightning_total
    };
    
    ESP_LOGI(TAG, "Sending WEATHER_ACK_ACK for seq=%d, lightning_total=%lu", 
             acked_seq, (unsigned long)lightning_total);
    return lora_send_secure(LORA_DEVICE_ID_BASE, MSG_TYPE_WEATHER_ACK_ACK,
                            (const uint8_t *)&ack_ack, sizeof(weather_ack_ack_payload_t));
}

bool lora_process_rx(void)
{
    uint8_t src_id, msg_type;
    uint8_t payload[64];
    
    int len = lora_receive_secure(&src_id, &msg_type, payload, sizeof(payload));
    if (len <= 0) return false;
    
    // Update recent RSSI for adaptive power
    recent_rssi = status.last_rssi;
    
    // Dispatch based on message type (sensor receives config, weather_ack)
    switch (msg_type) {
        case MSG_TYPE_CONFIG:
            if (len >= sizeof(config_payload_t) && config_callback) {
                const config_payload_t *config = (const config_payload_t *)payload;
                ESP_LOGI(TAG, "CONFIG received: interval=%d, power=%d, flags=0x%02X",
                         config->update_interval, config->tx_power, config->flags);
                
                // Handle locate flag (buzzer)
                if (config->flags & CFG_LOCATE_BUZZER) {
                    ESP_LOGI(TAG, "LOCATE BUZZER request received");
                    // Callback will handle the locate action
                }
                
                // Update high power mode setting
                high_power_enabled = (config->flags & CFG_HIGH_POWER) != 0;
                
                // Call user callback
                config_callback(config);
            }
            break;
            
        case MSG_TYPE_WEATHER_ACK:
            if (len >= sizeof(weather_ack_payload_t) && weather_ack_callback) {
                const weather_ack_payload_t *ack = (const weather_ack_payload_t *)payload;
                ESP_LOGI(TAG, "WEATHER_ACK received: seq=%d, base_rssi=%d, suggested_power=%d",
                         ack->acked_sequence, ack->base_rssi, ack->suggested_power);
                
                // Update power based on base station feedback (with high power limit)
                if (adaptive_power_enabled) {
                    uint8_t new_power = calculate_adaptive_power_limited(ack->base_rssi, high_power_enabled);
                    if (new_power != current_tx_power) {
                        current_tx_power = new_power;
                        lora_set_tx_power(current_tx_power);
                        ESP_LOGI(TAG, "Power adjusted to %d dBm (base RSSI=%d, high_power=%d)",
                                 current_tx_power, ack->base_rssi, high_power_enabled);
                    }
                }
                
                // Call user callback (handles lightning clearing)
                weather_ack_callback(ack);
            }
            break;
            
        // Legacy message types (deprecated but kept for transition)
        case MSG_TYPE_ACK:
            ESP_LOGW(TAG, "Legacy ACK received - use WEATHER_ACK instead");
            break;
            
        case MSG_TYPE_PING:
            ESP_LOGW(TAG, "Legacy PING received - use CONFIG with CFG_LOCATE_* flags instead");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown message type: 0x%02X", msg_type);
            return false;
    }
    
    return true;
}
