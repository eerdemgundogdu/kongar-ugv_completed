#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    float kp, ki, kd;
    float prevError;
    float integral;
    float integralLimit;

public:
    PIDController(float kp, float ki, float kd, float integralLimit = 100.0f);
    float compute(float setpoint, float measured, float dt);
    void reset();
};

#endif
