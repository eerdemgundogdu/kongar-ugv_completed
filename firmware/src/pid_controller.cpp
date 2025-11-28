#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd, float integralLimit)
    : kp(kp), ki(ki), kd(kd), integralLimit(integralLimit), prevError(0.0f), integral(0.0f) {}

float PIDController::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    
    float p = kp * error;
    
    integral += error * dt;
    integral = constrain(integral, -integralLimit, integralLimit);
    float i = ki * integral;
    
    float derivative = (error - prevError) / dt;
    float d = kd * derivative;
    
    prevError = error;
    
    return p + i + d;
}

void PIDController::reset() {
    prevError = 0.0f;
    integral = 0.0f;
}
