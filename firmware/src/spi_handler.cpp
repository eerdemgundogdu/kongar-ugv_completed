#include "spi_handler.h"

SPIHandler::SPIHandler(int csPin) : _csPin(csPin), _spiSettings(1000000, MSBFIRST, SPI_MODE0) {
}

void SPIHandler::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.begin();
}

uint8_t SPIHandler::readRegister(uint8_t reg) {
    uint8_t value = 0;
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80); // Read bit usually MSB
    value = SPI.transfer(0x00);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    return value;
}

void SPIHandler::writeRegister(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg & 0x7F); // Write bit usually 0
    SPI.transfer(value);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}

void SPIHandler::readBytes(uint8_t reg, uint8_t* buffer, uint8_t len) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80);
    for (int i = 0; i < len; i++) {
        buffer[i] = SPI.transfer(0x00);
    }
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
}

SPIHandler::RawData SPIHandler::readIMUData() {
    RawData data;
    // Dummy implementation for generic IMU
    // In a real scenario, you would read 12 or 14 bytes starting from ACCEL_XOUT_H
    // For now, we return 0s or random noise to simulate
    
    // Example reading 6 bytes (just as placeholder)
    // uint8_t buf[6];
    // readBytes(0x3B, buf, 6); 
    
    data.ax = 0;
    data.ay = 0;
    data.az = 9800; // ~1g
    data.gx = 0;
    data.gy = 0;
    data.gz = 0;
    
    return data;
}
