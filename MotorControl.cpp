// BLDC_MotorControl.h
#ifndef BLDC_MOTOR_CONTROL_H
#define BLDC_MOTOR_CONTROL_H

#include <Arduino.h>

class BLDCMotor {
private:
    uint8_t pwmPin;
    bool isArmed;
    bool isEnabled;
    int currentSpeed;
    int minPulse, maxPulse;
    
    // ESC params
    static const int PWM_FREQUENCY = 50;
    static const int PWM_RESOLUTION = 16;
    
public:
    BLDCMotor(uint8_t pin, int minPulse = 1000, int maxPulse = 2000);
    
    // Init & Arm
    void begin();
    bool arm();
    
    // Speed control
    void setSpeed(int speed);
    void setSpeedMicroseconds(int us);
    
    // Control
    void stop();
    void emergencyStop();
    
    // Status
    bool armed() const;
    bool enabled() const;
    int getSpeed() const;
    
    void setPulseRange(int min, int max);
};

class DualBLDCController {
private:
    BLDCMotor motor1, motor2;
    bool bothArmed;
    
public:
    DualBLDCController(BLDCMotor m1, BLDCMotor m2);
    
    void begin();
    bool armAll();
    
    // Individual control
    void setMotorSpeed(uint8_t motorNum, int speed);
    void stopMotor(uint8_t motorNum);
    
    // Coordinated control
    void setBothSpeeds(int speed1, int speed2);
    void setBothSpeeds(int speed);
    void stopAll();
    void emergencyStopAll();
    
    // Differential
    void differentialControl(int baseSpeed, int turn);
    
    // Ramping
    void rampToSpeed(uint8_t motorNum, int targetSpeed, int rampTimeMs);
    void rampBothToSpeed(int targetSpeed, int rampTimeMs);
    
    bool allArmed() const;
    void getStatus(int& speed1, int& speed2) const;
};

#endif

// BLDCMotor implementation
BLDCMotor::BLDCMotor(uint8_t pin, int minPulse, int maxPulse) 
    : pwmPin(pin), isArmed(false), isEnabled(true), currentSpeed(0),
      minPulse(minPulse), maxPulse(maxPulse) {
}

void BLDCMotor::begin() {
    // Setup PWM
    analogWriteFrequency(pwmPin, PWM_FREQUENCY);
    analogWriteResolution(PWM_RESOLUTION);
    
    stop();
    delay(100);
}

bool BLDCMotor::arm() {
    if (isArmed) return true;
    
    Serial.print("Arming motor on pin ");
    Serial.println(pwmPin);
    
    stop();
    delay(2000);
    
    isArmed = true;
    Serial.println("Motor armed successfully");
    return true;
}

void BLDCMotor::setSpeed(int speed) {
    if (!isArmed || !isEnabled) return;
    
    speed = constrain(speed, 0, 100);
    currentSpeed = speed;
    
    int pulseWidth = map(speed, 0, 100, minPulse, maxPulse);
    setSpeedMicroseconds(pulseWidth);
}

void BLDCMotor::setSpeedMicroseconds(int us) {
    if (!isArmed || !isEnabled) return;
    
    us = constrain(us, minPulse, maxPulse);
    
    float periodMicroseconds = 1000000.0 / PWM_FREQUENCY;
    int pwmValue = (us * 65535) / periodMicroseconds;
    
    analogWrite(pwmPin, pwmValue);
    
    currentSpeed = map(us, minPulse, maxPulse, 0, 100);
}

void BLDCMotor::stop() {
    setSpeedMicroseconds(minPulse);
    currentSpeed = 0;
}

void BLDCMotor::emergencyStop() {
    stop();
    isEnabled = false;
}

bool BLDCMotor::armed() const {
    return isArmed;
}

bool BLDCMotor::enabled() const {
    return isEnabled;
}

int BLDCMotor::getSpeed() const {
    return currentSpeed;
}

void BLDCMotor::setPulseRange(int min, int max) {
    minPulse = min;
    maxPulse = max;
}

// DualBLDCController implementation
DualBLDCController::DualBLDCController(BLDCMotor m1, BLDCMotor m2) 
    : motor1(m1), motor2(m2), bothArmed(false) {
}

void DualBLDCController::begin() {
    motor1.begin();
    motor2.begin();
}

bool DualBLDCController::armAll() {
    Serial.println("Arming both motors...");
    
    bool m1Armed = motor1.arm();
    delay(500);
    
    bool m2Armed = motor2.arm();
    delay(500);
    
    bothArmed = m1Armed && m2Armed;
    
    if (bothArmed) {
        Serial.println("Both motors armed successfully!");
    } else {
        Serial.println("Motor arming failed!");
    }
    
    return bothArmed;
}

void DualBLDCController::setMotorSpeed(uint8_t motorNum, int speed) {
    switch(motorNum) {
        case 1:
            motor1.setSpeed(speed);
            break;
        case 2:
            motor2.setSpeed(speed);
            break;
    }
}

void DualBLDCController::stopMotor(uint8_t motorNum) {
    setMotorSpeed(motorNum, 0);
}

void DualBLDCController::setBothSpeeds(int speed1, int speed2) {
    motor1.setSpeed(speed1);
    motor2.setSpeed(speed2);
}

void DualBLDCController::setBothSpeeds(int speed) {
    setBothSpeeds(speed, speed);
}

void DualBLDCController::stopAll() {
    motor1.stop();
    motor2.stop();
}

void DualBLDCController::emergencyStopAll() {
    motor1.emergencyStop();
    motor2.emergencyStop();
    bothArmed = false;
}

void DualBLDCController::differentialControl(int baseSpeed, int turn) {
    turn = constrain(turn, -100, 100);
    
    int leftSpeed = baseSpeed - turn;
    int rightSpeed = baseSpeed + turn;
    
    leftSpeed = constrain(leftSpeed, 0, 100);
    rightSpeed = constrain(rightSpeed, 0, 100);
    
    setBothSpeeds(leftSpeed, rightSpeed);
}

void DualBLDCController::rampToSpeed(uint8_t motorNum, int targetSpeed, int rampTimeMs) {
    int currentSpeed = (motorNum == 1) ? motor1.getSpeed() : motor2.getSpeed();
    int step = (targetSpeed > currentSpeed) ? 1 : -1;
    int steps = abs(targetSpeed - currentSpeed);
    
    if (steps == 0) return;
    
    int delayTime = rampTimeMs / steps;
    
    for (int i = 0; i < steps; i++) {
        currentSpeed += step;
        setMotorSpeed(motorNum, currentSpeed);
        delay(delayTime);
    }
}

void DualBLDCController::rampBothToSpeed(int targetSpeed, int rampTimeMs) {
    int currentSpeed1 = motor1.getSpeed();
    int currentSpeed2 = motor2.getSpeed();
    
    int steps = max(abs(targetSpeed - currentSpeed1), abs(targetSpeed - currentSpeed2));
    if (steps == 0) return;
    
    int delayTime = rampTimeMs / steps;
    
    for (int i = 0; i < steps; i++) {
        if (currentSpeed1 != targetSpeed) {
            currentSpeed1 += (targetSpeed > currentSpeed1) ? 1 : -1;
            motor1.setSpeed(currentSpeed1);
        }
        if (currentSpeed2 != targetSpeed) {
            currentSpeed2 += (targetSpeed > currentSpeed2) ? 1 : -1;
            motor2.setSpeed(currentSpeed2);
        }
        delay(delayTime);
    }
}

bool DualBLDCController::allArmed() const {
    return bothArmed;
}

void DualBLDCController::getStatus(int& speed1, int& speed2) const {
    speed1 = motor1.getSpeed();
    speed2 = motor2.getSpeed();
}

// Example usage
#define MOTOR1_PIN 2
#define MOTOR2_PIN 3

BLDCMotor motor1(MOTOR1_PIN, 1000, 2000);
BLDCMotor motor2(MOTOR2_PIN, 1000, 2000);

DualBLDCController controller(motor1, motor2);

void setup() {
    Serial.begin(115200);
    while (!Serial) ;
    
    Serial.println("BLDC Motor Controller Initializing...");
    
    controller.begin();
    
    if (!controller.armAll()) {
        Serial.println("Failed to arm motors! Check connections.");
        while(1);
    }
    
    Serial.println("System ready!");
    printCommands();
}

void loop() {
    if (Serial.available()) {
        handleSerialCommands();
    }
}

void handleSerialCommands() {
    char command = Serial.read();
    
    switch(command) {
        case 'w':
            controller.setBothSpeeds(60);
            Serial.println("Moving forward: 60%");
            break;
            
        case 's':
            controller.stopAll();
            Serial.println("Stopped");
            break;
            
        case 'a':
            controller.differentialControl(40, -30);
            Serial.println("Turning left");
            break;
            
        case 'd':
            controller.differentialControl(40, 30);
            Serial.println("Turning right");
            break;
            
        case 'x':
            controller.setBothSpeeds(40);
            Serial.println("Moving backward: 40%");
            break;
            
        case 'e':
            controller.emergencyStopAll();
            Serial.println("EMERGENCY STOP");
            break;
            
        case 'r':
            Serial.println("Ramping up...");
            controller.rampBothToSpeed(80, 3000);
            delay(2000);
            controller.rampBothToSpeed(0, 2000);
            Serial.println("Ramp test complete");
            break;
            
        case '?':
            int speed1, speed2;
            controller.getStatus(speed1, speed2);
            Serial.print("Motor1: "); Serial.print(speed1); Serial.println("%");
            Serial.print("Motor2: "); Serial.print(speed2); Serial.println("%");
            break;
    }
}

void printCommands() {
    Serial.println("\n=== BLDC Motor Control Commands ===");
    Serial.println("w - Move forward (60%)");
    Serial.println("s - Stop");
    Serial.println("a - Turn left");
    Serial.println("d - Turn right");
    Serial.println("x - Move backward (40%)");
    Serial.println("e - Emergency stop");
    Serial.println("r - Ramp test");
    Serial.println("? - Status");
    Serial.println("===================================");
}
