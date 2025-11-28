#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO085.h>

// Motor Pins
#define MOTOR_A_PWM 10
#define MOTOR_B_PWM 9
#define MOTOR_A_DIR 4
#define MOTOR_B_DIR 5

// PWM Range
#define MAX_PWM_VALUE 65535

// PID Constants
float Kp_speed = 1.0, Ki_speed = 0.1, Kd_speed = 0.5;
float Kp_heading = 1.0, Ki_heading = 0.1, Kd_heading = 0.5;

// State
int motorA_speed = 0;
int motorB_speed = 0;
int current_speed = 0;
float current_heading = 0;
float desired_heading = 0;

// BNO085 Object
Adafruit_BNO085 bno085 = Adafruit_BNO085();

// PID Vars
float prev_error_speed = 0, integral_speed = 0;
float prev_error_heading = 0, integral_heading = 0;

// Setup
void setup() {
  Serial.begin(115200);

  // Motor Pins
  pinMode(MOTOR_A_DIR, OUTPUT);
  pinMode(MOTOR_B_DIR, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);

  // Motors off
  analogWrite(MOTOR_A_PWM, motorA_speed);
  analogWrite(MOTOR_B_PWM, motorB_speed);

  // Init IMU
  if (!bno085.begin()) {
    Serial.println("Failed to find BNO085 sensor.");
    while (1);
  }

  // IMU Mode
  bno085.setOperationMode(Adafruit_BNO085::OPERATION_MODE_NDOF);
  
  Serial.println("BNO085 IMU initialized successfully.");
}

// Set Motor Speed
void setMotorSpeed(int motor, int speed) {
  if (speed > 0) {
    // Forward
    if (motor == 1) {
      digitalWrite(MOTOR_A_DIR, HIGH);
      analogWrite(MOTOR_A_PWM, speed);
    } else if (motor == 2) {
      digitalWrite(MOTOR_B_DIR, HIGH);
      analogWrite(MOTOR_B_PWM, speed);
    }
  } else if (speed < 0) {
    // Reverse
    if (motor == 1) {
      digitalWrite(MOTOR_A_DIR, LOW);
      analogWrite(MOTOR_A_PWM, -speed);
    } else if (motor == 2) {
      digitalWrite(MOTOR_B_DIR, LOW);
      analogWrite(MOTOR_B_PWM, -speed);
    }
  } else {
    // Stop
    if (motor == 1) {
      analogWrite(MOTOR_A_PWM, 0);
    } else if (motor == 2) {
      analogWrite(MOTOR_B_PWM, 0);
    }
  }
}

// Speed PID
int calculateSpeedPID(int desired_speed, int current_speed) {
  float error = desired_speed - current_speed;
  integral_speed += error;
  float derivative_speed = error - prev_error_speed;
  prev_error_speed = error;

  return (int)(Kp_speed * error + Ki_speed * integral_speed + Kd_speed * derivative_speed);
}

// Heading PID
float calculateHeadingPID(float desired_heading, float current_heading) {
  float error = desired_heading - current_heading;

  // Normalize error
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  integral_heading += error;
  float derivative_heading = error - prev_error_heading;
  prev_error_heading = error;

  return Kp_heading * error + Ki_heading * integral_heading + Kd_heading * derivative_heading;
}

// Loop
void loop() {
  // Get heading
  sensors_event_t event;
  bno085.getEvent(&event);
  current_heading = event.orientation.x;

  // Desired heading
  desired_heading = 0;

  // PID Calc
  float heading_adjustment = calculateHeadingPID(desired_heading, current_heading);

  // Adjust speeds
  motorA_speed = calculateSpeedPID(100, current_speed) + heading_adjustment;
  motorB_speed = calculateSpeedPID(100, current_speed) - heading_adjustment;

  // Limit PWM
  motorA_speed = constrain(motorA_speed, -MAX_PWM_VALUE, MAX_PWM_VALUE);
  motorB_speed = constrain(motorB_speed, -MAX_PWM_VALUE, MAX_PWM_VALUE);

  // Set speeds
  setMotorSpeed(1, motorA_speed);
  setMotorSpeed(2, motorB_speed);

  // Update speed
  current_speed = (motorA_speed + motorB_speed) / 2;

  // Debug
  Serial.print("Heading: ");
  Serial.print(current_heading);
  Serial.print(" | Desired Heading: ");
  Serial.print(desired_heading);
  Serial.print(" | Motor A Speed: ");
  Serial.print(motorA_speed);
  Serial.print(" | Motor B Speed: ");
  Serial.println(motorB_speed);

  delay(100);
}
