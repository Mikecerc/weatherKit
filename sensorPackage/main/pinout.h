/**
 * @file pinout.h
 * @brief Master pinout configuration for WeatherKit Sensor Package ESP32-S3
 * 
 * Board: ESP32-S3FH4R2 (4MB Flash, 2MB PSRAM)
 * 
 * All GPIO pin assignments should be defined here to avoid conflicts.
 * 
 * ESP32-S3 Notes:
 * - GPIO 0: Strapping pin (BOOT button), avoid for general use
 * - GPIO 3: Strapping pin, avoid for general use  
 * - GPIO 19-20: USB D-/D+, don't use (USB in use)
 * - GPIO 26-32: Connected to SPI flash, don't use
 * - GPIO 33-37: Connected to PSRAM, NOT AVAILABLE!
 * - GPIO 45-46: Strapping pins, avoid for general use
 */

#ifndef PINOUT_H
#define PINOUT_H

// =============================================================================
// I2C Bus (AHT20, AS3935 sensors)
// =============================================================================
#define PIN_I2C_SDA         8
#define PIN_I2C_SCL         9
#define I2C_FREQ_HZ         400000
#define I2C_HOST            I2C_NUM_0

// I2C Device Addresses
#define I2C_ADDR_AHT20      0x38    // or 0x77 depending on SDO pin
#define I2C_ADDR_AS3935     0x03    // Lightning sensor

// =============================================================================
// SPI Bus (HX710B sensor)
// =============================================================================
#define PIN_SPI_CLK         11

// =============================================================================
// Data Out Pins (HX710B, AS3935, AHT20 sensors)
// =============================================================================
#define PIN_AS3935_IRQ      7      //Interrupt pin for lightning sensor
#define PIN_HX710B_DOUT     10     //Data out for pressure snesor

//==============================================================================
//Chirper Pins
//==============================================================================
#define PIN_CHIRPER         6     // Chirper output pin

// =============================================================================
// LoRa Module (RA-02 / SX1278) - SPI
// Avoiding PSRAM GPIO 33-37
// =============================================================================
#define PIN_LORA_SCK        3       // SPI Clock
#define PIN_LORA_MISO       2       // SPI Master In, Slave Out  
#define PIN_LORA_MOSI       1       // SPI Master Out, Slave In
#define PIN_LORA_NSS        13      // Chip Select (active low)
#define PIN_LORA_RST        12      // Reset (active low)
#define PIN_LORA_DIO0       5       // Interrupt: RX done, TX done, CAD done
#define PIN_LORA_DIO1       4       // Interrupt: RX timeout (optional)
// DIO2-DIO5 optional, directly on SX1278 chip (under metal shield on RA-02)

// SPI Configuration for LoRa
#define LORA_SPI_HOST       SPI2_HOST
#define LORA_SPI_FREQ_HZ    1000000  // 1 MHz (max 10 MHz for SX1278)

// LoRa TX Power Levels (dBm)
// SX1278 PA_BOOST range: +2 to +17 dBm (or +20 with PA_HP)
#define LORA_TX_POWER_LOW   2       // +2 dBm - safe without antenna
#define LORA_TX_POWER_HIGH  17      // +17 dBm - requires antenna!

// =============================================================================
// Status LED (optional - for debugging/locate function)
// =============================================================================
#define PIN_STATUS_LED      21      // Status LED (if available)

// =============================================================================
// Battery Monitoring (ADC)
// =============================================================================
#define PIN_BATTERY_ADC     3       // Battery voltage divider input
#define BATTERY_ADC_CHANNEL ADC1_CHANNEL_2

// =============================================================================
// Available GPIOs (not currently used)
// =============================================================================
// GPIO 10: Available
// GPIO 11-18: Available (check for conflicts with peripherals)
// GPIO 38-44: Available
// GPIO 47-48: Available

#endif // PINOUT_H
