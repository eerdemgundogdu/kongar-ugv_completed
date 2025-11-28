#include "motor_driver.h"
#include "config.h"

MotorDriver::MotorDriver(uint8_t pwm, uint8_t dir, int minPulse, int maxPulse)
    : pwmPin(pwm), dirPin(dir), currentSpeed(0), minPulse(minPulse), maxPulse(maxPulse) {}

void MotorDriver::begin() {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    analogWriteFrequency(pwmPin, PWM_FREQUENCY);
    analogWriteResolution(PWM_RESOLUTION);
    stop();
}

void MotorDriver::setSpeed(int speed) {
    speed = constrain(speed, -100, 100);
    currentSpeed = speed;
    
    if (speed >= 0) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
        speed = -speed;
    }
    
    int pulseWidth = map(speed, 0, 100, minPulse, maxPulse);
    float periodMicroseconds = 1000000.0 / PWM_FREQUENCY;
    int pwmValue = (pulseWidth * 65535) / periodMicroseconds;
    
    analogWrite(pwmPin, pwmValue);
}

void MotorDriver::stop() {
    setSpeed(0);
}

int MotorDriver::getSpeed() const {
    return currentSpeed;
}
