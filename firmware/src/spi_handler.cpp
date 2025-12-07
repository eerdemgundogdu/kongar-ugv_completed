#include "spi_handler.h"

// MPU6050 Register Addresses
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43

SPIHandler::SPIHandler(int csPin) : _csPin(csPin), _spiSettings(1000000, MSBFIRST, SPI_MODE0) {
}

void SPIHandler::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    SPI.begin();
    
    // Wake up MPU6050 (clear sleep bit)
    writeRegister(MPU6050_PWR_MGMT_1, 0x00);
    delay(100);
    
    // Verify device
    uint8_t whoami = readRegister(MPU6050_WHO_AM_I);
    if (whoami == 0x68 || whoami == 0x98) {
        // Device found (MPU6050 or MPU9250)
    }
}

uint8_t SPIHandler::readRegister(uint8_t reg) {
    uint8_t value = 0;
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg | 0x80);
    value = SPI.transfer(0x00);
    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    return value;
}

void SPIHandler::writeRegister(uint8_t reg, uint8_t value) {
    SPI.beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    SPI.transfer(reg & 0x7F);
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
    uint8_t buffer[14];
    
    // Read 14 bytes starting from ACCEL_XOUT_H
    // Layout: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L, TEMP_H, TEMP_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
    readBytes(MPU6050_ACCEL_XOUT_H, buffer, 14);
    
    // Combine high and low bytes
    data.ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    data.ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    data.az = (int16_t)((buffer[4] << 8) | buffer[5]);
    // Skip temperature (buffer[6], buffer[7])
    data.gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    data.gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    data.gz = (int16_t)((buffer[12] << 8) | buffer[13]);
    
    return data;
}
