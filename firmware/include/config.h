#ifndef CONFIG_H
#define CONFIG_H

// Pin Definitions
#define MOTOR_LEFT_PWM 2
#define MOTOR_RIGHT_PWM 3
#define MOTOR_LEFT_DIR 4
#define MOTOR_RIGHT_DIR 5

#define ENCODER_LEFT_A 6
#define ENCODER_LEFT_B 7
#define ENCODER_RIGHT_A 8
#define ENCODER_RIGHT_B 9

// Safety
#define WATCHDOG_TIMEOUT_MS 500

// PID Parameters
#define PID_KP 1.0f
#define PID_KI 0.1f
#define PID_KD 0.05f

// PWM Settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 16
#define MIN_PULSE_US 1000
#define MAX_PULSE_US 2000

#endif
