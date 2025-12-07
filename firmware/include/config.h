#ifndef CONFIG_H
#define CONFIG_H

// Motor Pins
#define MOTOR_LEFT_PWM 2
#define MOTOR_RIGHT_PWM 3
#define MOTOR_LEFT_DIR 4
#define MOTOR_RIGHT_DIR 5

// Encoder Pins
#define ENCODER_LEFT_A 6
#define ENCODER_LEFT_B 7
#define ENCODER_RIGHT_A 8
#define ENCODER_RIGHT_B 9

// IMU SPI
#define IMU_CS_PIN 10
#define IMU_SPI_MOSI 11
#define IMU_SPI_MISO 12
#define IMU_SPI_SCK 13

// Battery
#define BATTERY_ADC_PIN A0
#define BATTERY_VOLTAGE_DIVIDER 11.0f
#define BATTERY_ADC_RESOLUTION 1023.0f
#define BATTERY_REFERENCE_VOLTAGE 3.3f
#define BATTERY_FULL_VOLTAGE 12.6f
#define BATTERY_EMPTY_VOLTAGE 9.6f

// Status
#define STATUS_LED_PIN 13

// Safety
#define WATCHDOG_TIMEOUT_MS 500
#define BATTERY_LOW_THRESHOLD 20.0f

// PID - tuned for typical UGV
#define PID_KP 2.0f
#define PID_KI 0.5f
#define PID_KD 0.1f
#define PID_OUTPUT_LIMIT 100.0f

// PWM
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 16
#define MIN_PULSE_US 1000
#define MAX_PULSE_US 2000

// Robot Physical - typical small UGV
#define WHEEL_RADIUS 0.065f
#define WHEEL_BASE 0.35f
#define TICKS_PER_REV 2400.0f
#define MAX_LINEAR_VEL 0.8f
#define MAX_ANGULAR_VEL 1.5f

// Timing
#define MOTOR_LOOP_HZ 50
#define SENSOR_LOOP_HZ 50
#define IMU_LOOP_HZ 50
#define COMMS_LOOP_MS 5
#define BATTERY_READ_INTERVAL_MS 1000

#endif
