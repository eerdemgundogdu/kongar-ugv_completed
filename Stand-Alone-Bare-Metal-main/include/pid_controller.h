#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    float compute(float target, float measured, float dt);
    void reset();

private:
    float kp, ki, kd;
    float prevError;
    float integral;
};

#endif
