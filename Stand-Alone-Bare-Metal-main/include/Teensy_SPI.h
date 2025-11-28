#ifndef TEENSY_SPI_H
#define TEENSY_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// SPI Pins (Teensy 4.1 - SPI0)
// SCK: Pin 13
// MOSI: Pin 11
// MISO: Pin 12
// CS: Pin 10 (Software Controlled)

void SPI_Init(void);
uint8_t SPI_Transfer(uint8_t data);
void SPI_CS_Low(void);
void SPI_CS_High(void);

#ifdef __cplusplus
}
#endif

#endif // TEENSY_SPI_H
