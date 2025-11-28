#include <Arduino_FreeRTOS.h>
#include <semphr.h>
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
SPIHandler spiHandler(10); // CS Pin 10

// Shared Data
SemaphoreHandle_t xMutexCmd;
float targetLinearVel = 0.0f;
float targetAngularVel = 0.0f;

// Task Handles
TaskHandle_t xTaskMotorHandle;
TaskHandle_t xTaskSensorsHandle;
TaskHandle_t xTaskCommsHandle;
TaskHandle_t xTaskIMUHandle;

// Task Prototypes
void TaskMotor(void *pvParameters);
void TaskSensors(void *pvParameters);
void TaskComms(void *pvParameters);
void TaskIMU(void *pvParameters);

// Robot Constants (Should be in config.h but defining here for now)
#define WHEEL_RADIUS 0.05f // meters
#define WHEEL_BASE 0.30f   // meters
#define TICKS_PER_REV 1000.0f

void setup() {
    Serial.begin(115200);
    
    motorLeft.begin();
    motorRight.begin();
    spiHandler.begin();
    
    xMutexCmd = xSemaphoreCreateMutex();
    
    // Create Tasks
    // Priorities: Motor (High) > IMU (Med) > Sensors (Med) > Comms (Low)
    xTaskCreate(TaskMotor, "Motor", 512, NULL, 4, &xTaskMotorHandle); // Increased stack for float math
    xTaskCreate(TaskIMU, "IMU", 256, NULL, 3, &xTaskIMUHandle);
    xTaskCreate(TaskSensors, "Sensors", 512, NULL, 2, &xTaskSensorsHandle);
    xTaskCreate(TaskComms, "Comms", 512, NULL, 1, &xTaskCommsHandle);
    
    vTaskStartScheduler();
}

void loop() {
    // Empty - FreeRTOS handles everything
}

// Global variables for Odometry sharing (could be protected by mutex but single writer/reader pattern often ok for simple floats)
// Using volatile for safety
volatile float currentLeftVel = 0.0f;
volatile float currentRightVel = 0.0f;
volatile float odomX = 0.0f;
volatile float odomY = 0.0f;
volatile float odomTheta = 0.0f;

void TaskMotor(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz Control Loop
    float dt = 0.02f;
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        if (watchdog.isTimedOut()) {
            motorLeft.stop();
            motorRight.stop();
            pidLeft.reset();
            pidRight.reset();
            continue;
        }
        
        float linear, angular;
        if (xSemaphoreTake(xMutexCmd, pdMS_TO_TICKS(5)) == pdTRUE) {
            linear = targetLinearVel;
            angular = targetAngularVel;
            xSemaphoreGive(xMutexCmd);
        } else {
            continue;
        }
        
        // Kinematics: Target Wheel Velocities (m/s)
        float targetLeftVel = linear - (angular * WHEEL_BASE / 2.0f);
        float targetRightVel = linear + (angular * WHEEL_BASE / 2.0f);
        
        // PID Compute
        // Output of PID is usually PWM or Voltage. Assuming -100 to 100 range for setSpeed
        // We need measured velocity. Let's use the global variables updated by TaskSensors
        // Note: There's a 1-cycle delay here, which is fine.
        
        float outputLeft = pidLeft.compute(targetLeftVel, currentLeftVel, dt);
        float outputRight = pidRight.compute(targetRightVel, currentRightVel, dt);
        
        // Feed Forward (optional but good for responsiveness)
        // float ffLeft = targetLeftVel * KV; 
        
        int pwmLeft = (int)(outputLeft * 100.0f); // Scaling factor needs tuning
        int pwmRight = (int)(outputRight * 100.0f);
        
        motorLeft.setSpeed(pwmLeft);
        motorRight.setSpeed(pwmRight);
    }
}

void TaskSensors(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz Update
    float dt = 0.02f;
    
    long prevLeftTicks = 0;
    long prevRightTicks = 0;
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        long currLeftTicks = encoderLeft.read();
        long currRightTicks = encoderRight.read();
        
        long deltaLeft = currLeftTicks - prevLeftTicks;
        long deltaRight = currRightTicks - prevRightTicks;
        
        prevLeftTicks = currLeftTicks;
        prevRightTicks = currRightTicks;
        
        // Calculate Velocities (m/s)
        // Distance per tick = (2 * PI * R) / TICKS_PER_REV
        float distPerTick = (2.0f * 3.14159f * WHEEL_RADIUS) / TICKS_PER_REV;
        
        float vLeft = (deltaLeft * distPerTick) / dt;
        float vRight = (deltaRight * distPerTick) / dt;
        
        // Update Globals for PID
        currentLeftVel = vLeft;
        currentRightVel = vRight;
        
        // Odometry Calculation
        float v = (vRight + vLeft) / 2.0f;
        float w = (vRight - vLeft) / WHEEL_BASE;
        
        odomTheta += w * dt;
        // Normalize Theta
        while (odomTheta > 3.14159f) odomTheta -= 2.0f * 3.14159f;
        while (odomTheta < -3.14159f) odomTheta += 2.0f * 3.14159f;
        
        odomX += v * cos(odomTheta) * dt;
        odomY += v * sin(odomTheta) * dt;
        
        // Send Odometry
        serialProtocol.sendOdom(odomX, odomY, odomTheta, v, w);
    }
}

void TaskIMU(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz IMU Update
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        SPIHandler::RawData data = spiHandler.readIMUData();
        
        // Convert raw to float (assuming scale factors)
        float ax = data.ax / 16384.0f; // Example scale for +/- 2g
        float ay = data.ay / 16384.0f;
        float az = data.az / 16384.0f;
        float gx = data.gx / 131.0f;   // Example scale for +/- 250dps
        float gy = data.gy / 131.0f;
        float gz = data.gz / 131.0f;
        
        serialProtocol.sendImu(ax, ay, az, gx, gy, gz);
    }
}

void TaskComms(void *pvParameters) {
    for (;;) {
        serialProtocol.update();
        
        if (serialProtocol.isCmdVelAvailable()) {
            CmdVelMsg cmd = serialProtocol.getCmdVel();
            
            if (xSemaphoreTake(xMutexCmd, pdMS_TO_TICKS(10)) == pdTRUE) {
                targetLinearVel = cmd.linear_x;
                targetAngularVel = cmd.angular_z;
                xSemaphoreGive(xMutexCmd);
                watchdog.feed();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
