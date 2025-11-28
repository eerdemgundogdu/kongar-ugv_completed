#ifndef TEENSY_ENCODER_H
#define TEENSY_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Encoder Pin Definitions (Teensy 4.1)
// Left Encoder: Pins 0, 1 (Using LPUART1 pins as GPIO/Timer for now? No, need dedicated QTimer pins)
// Let's use Pins 10, 11, 12, 13 which map to QTimer usually.
// Actually, let's check the schematic mapping.
// Pin 0: AD_B0_03 (QTIMER1_TIMER1)
// Pin 1: AD_B0_02 (QTIMER1_TIMER0)
// Pin 2: EMC_04
// Pin 3: EMC_05
// Pin 4: EMC_06

// For simplicity in this bare metal example, we will use:
// Left: QTIMER1 (Pins 0, 1) -> Need to disable LPUART1 on these pins!
// Right: QTIMER2 (Pins 6, 7) -> Pin 6 (AD_B0_10), Pin 7 (AD_B0_11)

void Encoder_Init(void);
int32_t Encoder_ReadLeft(void);
int32_t Encoder_ReadRight(void);

#ifdef __cplusplus
}
#endif

#endif // TEENSY_ENCODER_H
