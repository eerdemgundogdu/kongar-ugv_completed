#include <Arduino.h>
#include <TeensyThreads.h>
#include <Encoder.h>
#include "config.h"
#include "motor_driver.h"
#include "pid_controller.h"
#include "watchdog.h"
#include "serial_protocol.h"
#include "spi_handler.h"

// Hardware Objects
MotorDriver motorLeft(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR);
MotorDriver motorRight(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR);

Encoder encoderLeft(ENCODER_LEFT_A, ENCODER_LEFT_B);
Encoder encoderRight(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

PIDController pidLeft(PID_KP, PID_KI, PID_KD);
PIDController pidRight(PID_KP, PID_KI, PID_KD);

Watchdog watchdog(WATCHDOG_TIMEOUT_MS);
SerialProtocol serialProtocol(Serial);
SPIHandler spiHandler(IMU_CS_PIN);

// Shared Data with mutex protection
Threads::Mutex cmdMutex;
volatile float targetLinearVel = 0.0f;
volatile float targetAngularVel = 0.0f;

// Odometry data
volatile float currentLeftVel = 0.0f;
volatile float currentRightVel = 0.0f;
volatile float odomX = 0.0f;
volatile float odomY = 0.0f;
volatile float odomTheta = 0.0f;

// Battery data
volatile float batteryVoltage = 12.0f;
volatile float batteryPercent = 100.0f;

// Thread IDs
int motorThreadId;
int sensorsThreadId;
int commsThreadId;
int imuThreadId;
int batteryThreadId;

float readBatteryVoltage() {
    int rawAdc = analogRead(BATTERY_ADC_PIN);
    float voltage = (rawAdc / BATTERY_ADC_RESOLUTION) * BATTERY_REFERENCE_VOLTAGE * BATTERY_VOLTAGE_DIVIDER;
    return voltage;
}

float voltageToPercent(float voltage) {
    float percent = ((voltage - BATTERY_EMPTY_VOLTAGE) / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE)) * 100.0f;
    return constrain(percent, 0.0f, 100.0f);
}

void motorThread() {
    const unsigned long interval = 1000 / MOTOR_LOOP_HZ;
    float dt = 1.0f / MOTOR_LOOP_HZ;
    
    while (1) {
        unsigned long start = millis();
        
        if (watchdog.isTimedOut()) {
            motorLeft.stop();
            motorRight.stop();
            pidLeft.reset();
            pidRight.reset();
            threads.delay(interval);
            continue;
        }
        
        float linear, angular;
        {
            Threads::Scope lock(cmdMutex);
            linear = targetLinearVel;
            angular = targetAngularVel;
        }
        
        // Clamp velocities
        linear = constrain(linear, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
        angular = constrain(angular, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);
        
        // Differential drive kinematics
        float targetLeftVel = linear - (angular * WHEEL_BASE / 2.0f);
        float targetRightVel = linear + (angular * WHEEL_BASE / 2.0f);
        
        // PID compute
        float outputLeft = pidLeft.compute(targetLeftVel, currentLeftVel, dt);
        float outputRight = pidRight.compute(targetRightVel, currentRightVel, dt);
        
        // Scale and apply
        int pwmLeft = (int)constrain(outputLeft * 100.0f, -100.0f, 100.0f);
        int pwmRight = (int)constrain(outputRight * 100.0f, -100.0f, 100.0f);
        
        motorLeft.setSpeed(pwmLeft);
        motorRight.setSpeed(pwmRight);
        
        unsigned long elapsed = millis() - start;
        if (elapsed < interval) {
            threads.delay(interval - elapsed);
        }
    }
}

void sensorsThread() {
    const unsigned long interval = 1000 / SENSOR_LOOP_HZ;
    float dt = 1.0f / SENSOR_LOOP_HZ;
    
    long prevLeftTicks = 0;
    long prevRightTicks = 0;
    
    while (1) {
        unsigned long start = millis();
        
        long currLeftTicks = encoderLeft.read();
        long currRightTicks = encoderRight.read();
        
        long deltaLeft = currLeftTicks - prevLeftTicks;
        long deltaRight = currRightTicks - prevRightTicks;
        
        prevLeftTicks = currLeftTicks;
        prevRightTicks = currRightTicks;
        
        // Calculate Velocities (m/s)
        float distPerTick = (2.0f * PI * WHEEL_RADIUS) / TICKS_PER_REV;
        
        float vLeft = (deltaLeft * distPerTick) / dt;
        float vRight = (deltaRight * distPerTick) / dt;
        
        // Update Globals for PID
        currentLeftVel = vLeft;
        currentRightVel = vRight;
        
        // Odometry Calculation
        float v = (vRight + vLeft) / 2.0f;
        float w = (vRight - vLeft) / WHEEL_BASE;
        
        odomTheta += w * dt;
        // Normalize Theta to [-PI, PI]
        while (odomTheta > PI) odomTheta -= 2.0f * PI;
        while (odomTheta < -PI) odomTheta += 2.0f * PI;
        
        odomX += v * cos(odomTheta) * dt;
        odomY += v * sin(odomTheta) * dt;
        
        // Send Odometry
        serialProtocol.sendOdom(odomX, odomY, odomTheta, v, w);
        
        unsigned long elapsed = millis() - start;
        if (elapsed < interval) {
            threads.delay(interval - elapsed);
        }
    }
}

void imuThread() {
    const unsigned long interval = 1000 / IMU_LOOP_HZ;
    
    while (1) {
        unsigned long start = millis();
        
        SPIHandler::RawData data = spiHandler.readIMUData();
        
        // Convert raw to float (MPU6050 scale factors)
        float ax = data.ax / 16384.0f;  // ±2g
        float ay = data.ay / 16384.0f;
        float az = data.az / 16384.0f;
        float gx = data.gx / 131.0f;    // ±250°/s
        float gy = data.gy / 131.0f;
        float gz = data.gz / 131.0f;
        
        serialProtocol.sendImu(ax, ay, az, gx, gy, gz);
        
        unsigned long elapsed = millis() - start;
        if (elapsed < interval) {
            threads.delay(interval - elapsed);
        }
    }
}

void batteryThread() {
    while (1) {
        batteryVoltage = readBatteryVoltage();
        batteryPercent = voltageToPercent(batteryVoltage);
        
        // Send battery status
        serialProtocol.sendBattery(batteryPercent);
        
        // Low battery warning
        if (batteryPercent < BATTERY_LOW_THRESHOLD) {
            digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
        } else {
            digitalWrite(STATUS_LED_PIN, HIGH);
        }
        
        threads.delay(BATTERY_READ_INTERVAL_MS);
    }
}

void commsThread() {
    while (1) {
        serialProtocol.update();
        
        if (serialProtocol.isCmdVelAvailable()) {
            CmdVelMsg cmd = serialProtocol.getCmdVel();
            
            {
                Threads::Scope lock(cmdMutex);
                targetLinearVel = cmd.linear_x;
                targetAngularVel = cmd.angular_z;
            }
            watchdog.feed();
        }
        
        threads.delay(COMMS_LOOP_MS);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    // Pin modes
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(BATTERY_ADC_PIN, INPUT);
    analogReadResolution(10);
    
    // Initialize hardware
    motorLeft.begin();
    motorRight.begin();
    spiHandler.begin();
    
    digitalWrite(STATUS_LED_PIN, HIGH);
    
    // Start threads with appropriate stack sizes
    motorThreadId = threads.addThread(motorThread, 0, 4096);
    sensorsThreadId = threads.addThread(sensorsThread, 0, 4096);
    imuThreadId = threads.addThread(imuThread, 0, 2048);
    batteryThreadId = threads.addThread(batteryThread, 0, 2048);
    commsThreadId = threads.addThread(commsThread, 0, 4096);
    
    Serial.println("UGV Firmware v2.0 Ready");
    Serial.print("Threads started: ");
    Serial.println(threads.threadsCount());
}

void loop() {
    threads.yield();
}
