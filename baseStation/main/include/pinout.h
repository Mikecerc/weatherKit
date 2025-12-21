/**
 * @file pinout.h
 * @brief Master pinout configuration for WeatherKit ESP32-S3
 * 
 * Board: Waveshare ESP32-S3-Zero
 * 
 * All GPIO pin assignments should be defined here
 * 
 */

#ifndef PINOUT_H
#define PINOUT_H

// =============================================================================
// I2C Bus (Display, BME280, AS3935)
// =============================================================================
#define PIN_I2C_SDA         1
#define PIN_I2C_SCL         2
#define I2C_FREQ_HZ         400000
#define I2C_HOST            I2C_NUM_0

// I2C Device Addresses
#define I2C_ADDR_OLED       0x3C
#define I2C_ADDR_BME280     0x76    // or 0x77 depending on SDO pin
#define I2C_ADDR_AS3935     0x03    // Lightning sensor

// =============================================================================
// Buttons (Active Low - pressed = GND)
// =============================================================================
#define PIN_BUTTON_LEFT     13
#define PIN_BUTTON_RIGHT    12

// =============================================================================
// LoRa Module (RA-02 / SX1278) - SPI
// Waveshare ESP32-S3-Zero compatible pins (avoiding PSRAM GPIO 33-37)
// =============================================================================
#define PIN_LORA_SCK        10//10      // SPI Clock
#define PIN_LORA_MISO       9//9      // SPI Master In, Slave Out  
#define PIN_LORA_MOSI       8//8     // SPI Master Out, Slave In
#define PIN_LORA_NSS        7//7       // Chip Select (active low)
#define PIN_LORA_RST        6//11     // Reset (active low)
#define PIN_LORA_DIO0       5// 5      // Interrupt: RX done, TX done, CAD done
#define PIN_LORA_DIO1       4//6       // Interrupt: RX timeout (optional)
// DIO2-DIO5 optional, directly on SX1278 chip (under metal shield on RA-02)

// SPI Configuration for LoRa
#define LORA_SPI_HOST       SPI2_HOST
#define LORA_SPI_FREQ_HZ    1000000  // 1 MHz (max 10 MHz for SX1278)

// LoRa TX Power Levels (dBm)
// SX1278 PA_BOOST range: +2 to +17 dBm (or +20 with PA_HP)
#define LORA_TX_POWER_LOW   2       // +2 dBm - safe without antenna
#define LORA_TX_POWER_HIGH  17      // +17 dBm - requires antenna

// =============================================================================
// Onboard RGB LED (WS2812)
// =============================================================================
#define PIN_RGB_LED         21      // WS2812 on ESP32-S3-Zero

#endif // PINOUT_H
