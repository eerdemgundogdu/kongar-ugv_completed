#include "Teensy_UART.h"
#include "Teensy_PWM.h"
#include "Teensy_Encoder.h"
#include "Teensy_SPI.h"
#include "pid_controller.h"
#include "serial_protocol.h"
#include <math.h>

// Globals
PIDController pidLeft(1.0, 0.0, 0.0);
PIDController pidRight(1.0, 0.0, 0.0);
SerialProtocol serialProtocol;

volatile float targetLinearVel = 0.0f;
volatile float targetAngularVel = 0.0f;

volatile float odomX = 0.0f;
volatile float odomY = 0.0f;
volatile float odomTheta = 0.0f;

// Robot Configuration
#define WHEEL_RADIUS 0.05f
#define WHEEL_BASE 0.30f
#define TICKS_PER_REV 1000.0f

void TaskMotor(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);
    
    float prevLeftVel = 0;
    float prevRightVel = 0;
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        float v = targetLinearVel;
        float w = targetAngularVel;
        
        float vLeft = v - (w * WHEEL_BASE / 2.0f);
        float vRight = v + (w * WHEEL_BASE / 2.0f);
        
        PWM_SetDutyLeft(vLeft);  
        PWM_SetDutyRight(vRight);
    }
}

void TaskSensors(void *pvParameters) {
    (void)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);
    
    int32_t prevLeft = 0;
    int32_t prevRight = 0;
    
    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        int32_t currLeft = Encoder_ReadLeft();
        int32_t currRight = Encoder_ReadRight();
        
        int32_t dLeft = currLeft - prevLeft;
        int32_t dRight = currRight - prevRight;
        
        prevLeft = currLeft;
        prevRight = currRight;
        
        float distPerTick = (2.0f * 3.14159f * WHEEL_RADIUS) / TICKS_PER_REV;
        float vLeft = dLeft * distPerTick * 50.0f;
        float vRight = dRight * distPerTick * 50.0f;
        
        float v = (vRight + vLeft) / 2.0f;
        float w = (vRight - vLeft) / WHEEL_BASE;
        
        odomTheta += w * 0.02f;
        odomX += v * cos(odomTheta) * 0.02f;
        odomY += v * sin(odomTheta) * 0.02f;
        
        serialProtocol.sendOdom(odomX, odomY, odomTheta, v, w);
    }
}

void TaskComms(void *pvParameters) {
    (void)pvParameters;
    
    for (;;) {
        // Simple parser loop
        char c = LPUART1_Read_Char(); // Blocks
        
        // In a real app, we would feed this to serialProtocol.parse(c)
        // For now, let's just echo for testing or implement basic command parsing
        
        // If we receive 'W', move forward
        if (c == 'W') targetLinearVel = 0.5f;
        if (c == 'S') targetLinearVel = 0.0f;
        
        // Also send heartbeat
        serialProtocol.sendHeartbeat();
    }
}

int main(void) {
    // Init Drivers
    LPUART1_INIT(115200);
    PWM_Init();
    Encoder_Init();
    SPI_Init();
    
    // Create Tasks
    xTaskCreate(TaskMotor, "Motor", 512, NULL, 3, NULL);
    xTaskCreate(TaskSensors, "Sensors", 512, NULL, 2, NULL);
    xTaskCreate(TaskComms, "Comms", 512, NULL, 1, NULL);
    
    vTaskStartScheduler();
    
    while(1);
    return 0;
}
