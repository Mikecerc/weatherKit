#ifndef LIGHTNING_H
#define LIGHTNING_H

#include <Wire.h>
#include <stdint.h>

class AS3935 {
private:
    static const uint8_t AS3935_ADDR = 0x03;
    
    // Register addresses
    static const uint8_t REG_AFE_GAIN = 0x00;
    static const uint8_t REG_THRESHOLD = 0x01;
    static const uint8_t REG_INT = 0x03;
    static const uint8_t REG_DISTANCE = 0x07;
    static const uint8_t REG_DISTURBER = 0x08;
    
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);

public:
    AS3935();
    void begin();
    uint8_t getInterruptReason();
    uint8_t getDistance();
    uint8_t getSignalStrength();
    bool isLightningDetected();
    bool isDisturbanceDetected();
};

#endif