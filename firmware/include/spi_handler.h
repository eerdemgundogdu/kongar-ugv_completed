#ifndef SPI_HANDLER_H
#define SPI_HANDLER_H

#include <Arduino.h>
#include <SPI.h>

class SPIHandler {
public:
    SPIHandler(int csPin);
    void begin();
    
    // Generic register read/write
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void readBytes(uint8_t reg, uint8_t* buffer, uint8_t len);
    
    // Placeholder for specific IMU data reading
    // Assuming a standard 6-axis IMU structure
    struct RawData {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;
    };
    
    RawData readIMUData();

private:
    int _csPin;
    SPISettings _spiSettings;
};

#endif
