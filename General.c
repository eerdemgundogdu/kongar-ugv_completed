#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

// IMU Libs
#include <MPU6050.h>

// GPS Lib
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Motor Pins
#define MOTOR_LEFT_PWM 3
#define MOTOR_LEFT_DIR1 4
#define MOTOR_LEFT_DIR2 5
#define MOTOR_RIGHT_PWM 6
#define MOTOR_RIGHT_DIR1 7
#define MOTOR_RIGHT_DIR2 8

// Sensor Pins
#define GPS_RX_PIN 14
#define GPS_TX_PIN 15
#define ULTRASONIC_TRIG 9
#define ULTRASONIC_ECHO 10
#define SERVO_PIN 11

// RPi Comm
#define RPI_SERIAL Serial1

// States
enum SystemState {
  INIT,
  AUTONOMOUS_NAV,
  OBSTACLE_AVOIDANCE,
  TARGET_TRACKING,
  EMERGENCY_STOP
};

// Nav Data
struct NavigationData {
  double currentLat;
  double currentLon;
  double targetLat;
  double targetLon;
  float heading;
  float targetHeading;
  float distance;
  bool gpsValid;
};

struct IMUData {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pitch, roll, yaw;
  unsigned long lastUpdate;
};

struct ObstacleData {
  float frontDistance;
  float leftDistance;
  float rightDistance;
  bool obstacleDetected;
  unsigned long lastScan;
};

struct RPiCommand {
  char command;
  float param1;
  float param2;
  float param3;
  bool valid;
};

// Globals
MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);
Servo scanServo;

SystemState currentState = INIT;
NavigationData navData = {0};
IMUData imuData = {0};
ObstacleData obstacles = {0};
RPiCommand rpiCmd = {0};

// Params
const float KP_HEADING = 2.0;
const float KI_HEADING = 0.1;
const float KD_HEADING = 0.5;
const float MAX_SPEED = 200;
const float MIN_OBSTACLE_DISTANCE = 30.0; // cm
const unsigned long SENSOR_UPDATE_INTERVAL = 50; // ms
const unsigned long GPS_UPDATE_INTERVAL = 1000; // ms

// PID vars
float headingError = 0;
float lastHeadingError = 0;
float headingIntegral = 0;
unsigned long lastPIDUpdate = 0;

void setup() {
  Serial.begin(115200);
  RPI_SERIAL.begin(115200);
  ss.begin(9600);
  
  // Init I2C
  Wire.begin();
  mpu.initialize();
  
  // Motor pins
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Ultrasonic pins
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  
  // Servo
  scanServo.attach(SERVO_PIN);
  scanServo.write(90);
  
  // Check IMU
  if (!mpu.testConnection()) {
    Serial.println("IMU connection failed");
  } else {
    Serial.println("IMU initialized successfully");
  }
  
  Serial.println("UGV Control System Initialized");
  currentState = AUTONOMOUS_NAV;
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update sensors
  updateGPS();
  updateIMU(currentTime);
  updateObstacles(currentTime);
  updateRPiCommunication();
  
  // FSM
  switch (currentState) {
    case INIT:
      initializeSystem();
      break;
      
    case AUTONOMOUS_NAV:
      autonomousNavigation();
      break;
      
    case OBSTACLE_AVOIDANCE:
      avoidObstacles();
      break;
      
    case TARGET_TRACKING:
      trackTarget();
      break;
      
    case EMERGENCY_STOP:
      emergencyStop();
      break;
  }
  
  // Telemetry
  sendTelemetry();
  
  delay(10);
}

void updateGPS() {
  static unsigned long lastGPSUpdate = 0;
  
  if (millis() - lastGPSUpdate > GPS_UPDATE_INTERVAL) {
    while (ss.available() > 0) {
      if (gps.encode(ss.read())) {
        if (gps.location.isValid()) {
          navData.currentLat = gps.location.lat();
          navData.currentLon = gps.location.lng();
          navData.gpsValid = true;
          
          // Calc dist & bearing
          if (navData.targetLat != 0 && navData.targetLon != 0) {
            navData.distance = gps.distanceBetween(
              navData.currentLat, navData.currentLon,
              navData.targetLat, navData.targetLon
            );
            navData.targetHeading = gps.courseTo(
              navData.currentLat, navData.currentLon,
              navData.targetLat, navData.targetLon
            );
          }
        } else {
          navData.gpsValid = false;
        }
      }
    }
    lastGPSUpdate = millis();
  }
}

void updateIMU(unsigned long currentTime) {
  if (currentTime - imuData.lastUpdate > SENSOR_UPDATE_INTERVAL) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert units
    imuData.accelX = ax / 16384.0;
    imuData.accelY = ay / 16384.0;
    imuData.accelZ = az / 16384.0;
    imuData.gyroX = gx / 131.0;
    imuData.gyroY = gy / 131.0;
    imuData.gyroZ = gz / 131.0;
    
    // Complementary filter
    float dt = (currentTime - imuData.lastUpdate) / 1000.0;
    imuData.pitch = atan2(imuData.accelY, sqrt(imuData.accelX * imuData.accelX + imuData.accelZ * imuData.accelZ)) * 180 / PI;
    imuData.roll = atan2(-imuData.accelX, imuData.accelZ) * 180 / PI;
    
    // Yaw integration
    imuData.yaw += imuData.gyroZ * dt;
    if (imuData.yaw > 360) imuData.yaw -= 360;
    if (imuData.yaw < 0) imuData.yaw += 360;
    
    navData.heading = imuData.yaw;
    imuData.lastUpdate = currentTime;
  }
}

void updateObstacles(unsigned long currentTime) {
  static int scanPosition = 0;
  static unsigned long lastScan = 0;
  
  if (currentTime - lastScan > 100) { // 100ms
    // Move servo
    int servoPositions[] = {45, 90, 135};
    scanServo.write(servoPositions[scanPosition]);
    delay(50);
    
    // Read dist
    float distance = readUltrasonicDistance();
    
    // Store dist
    switch (scanPosition) {
      case 0: obstacles.leftDistance = distance; break;
      case 1: obstacles.frontDistance = distance; break;
      case 2: obstacles.rightDistance = distance; break;
    }
    
    scanPosition = (scanPosition + 1) % 3;
    
    // Check obstacles
    obstacles.obstacleDetected = (obstacles.frontDistance < MIN_OBSTACLE_DISTANCE);
    
    lastScan = currentTime;
  }
}

float readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999;
  
  float distance = duration * 0.034 / 2; // cm
  return distance;
}

void updateRPiCommunication() {
  if (RPI_SERIAL.available()) {
    String message = RPI_SERIAL.readStringUntil('\n');
    parseRPiCommand(message);
  }
}

void parseRPiCommand(String message) {
  // Format: "CMD:param1,param2,param3"
  int colonIndex = message.indexOf(':');
  if (colonIndex > 0) {
    rpiCmd.command = message.charAt(0);
    String params = message.substring(colonIndex + 1);
    
    // Parse params
    int comma1 = params.indexOf(',');
    int comma2 = params.indexOf(',', comma1 + 1);
    
    if (comma1 > 0) {
      rpiCmd.param1 = params.substring(0, comma1).toFloat();
      if (comma2 > comma1) {
        rpiCmd.param2 = params.substring(comma1 + 1, comma2).toFloat();
        rpiCmd.param3 = params.substring(comma2 + 1).toFloat();
      } else {
        rpiCmd.param2 = params.substring(comma1 + 1).toFloat();
      }
    } else {
      rpiCmd.param1 = params.toFloat();
    }
    
    rpiCmd.valid = true;
    processRPiCommand();
  }
}

void processRPiCommand() {
  if (!rpiCmd.valid) return;
  
  switch (rpiCmd.command) {
    case 'G': // Go to GPS
      navData.targetLat = rpiCmd.param1;
      navData.targetLon = rpiCmd.param2;
      currentState = AUTONOMOUS_NAV;
      break;
      
    case 'T': // Track target
      currentState = TARGET_TRACKING;
      break;
      
    case 'S': // Stop
      currentState = EMERGENCY_STOP;
      break;
      
    case 'M': // Manual
      setMotorSpeeds(rpiCmd.param1, rpiCmd.param2);
      break;
      
    case 'A': // Resume auto
      currentState = AUTONOMOUS_NAV;
      break;
  }
  
  rpiCmd.valid = false;
}

void autonomousNavigation() {
  if (!navData.gpsValid || navData.distance < 2.0) {
    setMotorSpeeds(0, 0);
    return;
  }
  
  // Check obstacles
  if (obstacles.obstacleDetected) {
    currentState = OBSTACLE_AVOIDANCE;
    return;
  }
  
  // PID heading
  float targetSpeed = calculateNavigationSpeed();
  float steeringCorrection = calculateSteeringPID();
  
  float leftSpeed = targetSpeed - steeringCorrection;
  float rightSpeed = targetSpeed + steeringCorrection;
  
  // Constrain
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  
  setMotorSpeeds(leftSpeed, rightSpeed);
}

void avoidObstacles() {
  // Simple avoidance
  if (obstacles.frontDistance < MIN_OBSTACLE_DISTANCE) {
    if (obstacles.leftDistance > obstacles.rightDistance) {
      // Turn left
      setMotorSpeeds(-100, 100);
    } else {
      // Turn right
      setMotorSpeeds(100, -100);
    }
  } else {
    // Move forward slowly
    setMotorSpeeds(80, 80);
  }
  
  // Return to auto
  if (obstacles.frontDistance > MIN_OBSTACLE_DISTANCE * 1.5) {
    currentState = AUTONOMOUS_NAV;
  }
}

void trackTarget() {
  // Coordinated with RPi vision
  
  if (rpiCmd.valid && rpiCmd.command == 'T') {
    float xOffset = rpiCmd.param1; // -1 to 1
    float targetDistance = rpiCmd.param2;
    
    // Proportional control
    float turnRate = xOffset * 100;
    float forwardSpeed = 100;
    
    if (targetDistance < 50) { // Stop
      setMotorSpeeds(0, 0);
    } else {
      setMotorSpeeds(forwardSpeed - turnRate, forwardSpeed + turnRate);
    }
  } else {
    // Stop and scan
    setMotorSpeeds(0, 0);
  }
}

void emergencyStop() {
  setMotorSpeeds(0, 0);
}

float calculateNavigationSpeed() {
  // Speed based on dist
  float baseSpeed = MAX_SPEED * 0.7;
  
  // Reduce if close
  if (navData.distance < 10.0) {
    baseSpeed *= (navData.distance / 10.0);
  }
  
  // Reduce if obstacles
  if (obstacles.frontDistance < MIN_OBSTACLE_DISTANCE * 2) {
    baseSpeed *= 0.5;
  }
  
  return baseSpeed;
}

float calculateSteeringPID() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastPIDUpdate) / 1000.0;
  
  if (dt < 0.001) return 0;
  
  // Heading error
  headingError = navData.targetHeading - navData.heading;
  
  // Normalize error
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;
  
  // PID
  headingIntegral += headingError * dt;
  headingIntegral = constrain(headingIntegral, -50, 50);
  
  float headingDerivative = (headingError - lastHeadingError) / dt;
  
  float output = KP_HEADING * headingError + 
                 KI_HEADING * headingIntegral + 
                 KD_HEADING * headingDerivative;
  
  lastHeadingError = headingError;
  lastPIDUpdate = currentTime;
  
  return constrain(output, -MAX_SPEED/2, MAX_SPEED/2);
}

void setMotorSpeeds(float leftSpeed, float rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(MOTOR_LEFT_PWM, constrain(leftSpeed, 0, 255));
  
  // Right motor
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(MOTOR_RIGHT_PWM, constrain(rightSpeed, 0, 255));
}

void sendTelemetry() {
  static unsigned long lastTelemetry = 0;
  
  if (millis() - lastTelemetry > 500) { // 500ms
    // Send JSON
    RPI_SERIAL.print("{");
    RPI_SERIAL.print("\"lat\":" + String(navData.currentLat, 6) + ",");
    RPI_SERIAL.print("\"lon\":" + String(navData.currentLon, 6) + ",");
    RPI_SERIAL.print("\"heading\":" + String(navData.heading, 2) + ",");
    RPI_SERIAL.print("\"distance\":" + String(navData.distance, 2) + ",");
    RPI_SERIAL.print("\"front_dist\":" + String(obstacles.frontDistance, 1) + ",");
    RPI_SERIAL.print("\"left_dist\":" + String(obstacles.leftDistance, 1) + ",");
    RPI_SERIAL.print("\"right_dist\":" + String(obstacles.rightDistance, 1) + ",");
    RPI_SERIAL.print("\"state\":" + String(currentState) + ",");
    RPI_SERIAL.print("\"gps_valid\":" + String(navData.gpsValid ? 1 : 0));
    RPI_SERIAL.println("}");
    
    lastTelemetry = millis();
  }
}

void initializeSystem() {
  // Init sequence
  static int initStep = 0;
  static unsigned long initTimer = 0;
  
  if (millis() - initTimer > 1000) {
    switch (initStep) {
      case 0:
        Serial.println("Initializing IMU...");
        break;
      case 1:
        Serial.println("Waiting for GPS fix...");
        break;
      case 2:
        Serial.println("Testing motors...");
        setMotorSpeeds(100, 100);
        delay(500);
        setMotorSpeeds(0, 0);
        break;
      case 3:
        Serial.println("System ready!");
        currentState = AUTONOMOUS_NAV;
        return;
    }
    initStep++;
    initTimer = millis();
  }
}
