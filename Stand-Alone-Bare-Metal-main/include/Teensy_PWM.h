#ifndef TEENSY_PWM_H
#define TEENSY_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Motor Pin Definitions
// Left Motor:  Pin 2 (FlexPWM4 Module 2 Channel A)
// Right Motor: Pin 3 (FlexPWM4 Module 2 Channel B)
// Dir Pins: 4, 5

void PWM_Init(void);
void PWM_SetDutyLeft(float duty);  // -1.0 to 1.0
void PWM_SetDutyRight(float duty); // -1.0 to 1.0

#ifdef __cplusplus
}
#endif

#endif // TEENSY_PWM_H
