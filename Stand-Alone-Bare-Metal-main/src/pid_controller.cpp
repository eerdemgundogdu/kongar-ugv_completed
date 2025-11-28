#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), prevError(0.0f), integral(0.0f) {}

float PIDController::compute(float target, float measured, float dt) {
    float error = target - measured;
    integral += error * dt;
    
    // Anti-windup
    if (integral > 10.0f) integral = 10.0f;
    if (integral < -10.0f) integral = -10.0f;
    
    float derivative = (error - prevError) / dt;
    prevError = error;
    
    return (kp * error) + (ki * integral) + (kd * derivative);
}

void PIDController::reset() {
    prevError = 0.0f;
    integral = 0.0f;
}
