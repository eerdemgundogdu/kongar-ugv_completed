#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
private:
    uint8_t pwmPin;
    uint8_t dirPin;
    int currentSpeed;
    int minPulse, maxPulse;

public:
    MotorDriver(uint8_t pwm, uint8_t dir, int minPulse = 1000, int maxPulse = 2000);
    void begin();
    void setSpeed(int speed); // -100 to 100
    void stop();
    int getSpeed() const;
};

#endif
