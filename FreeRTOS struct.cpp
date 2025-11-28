// main.ino or main.cpp

#include <Arduino_FreeRTOS.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "motor_controller.h"
#include "imu_manager.h"
#include "odometry.h"

ros::NodeHandle nh;

// Tasks
TaskHandle_t xMotorControlTask;
TaskHandle_t xImuTask;
TaskHandle_t xOdometryTask;
TaskHandle_t xRosCommunicationTask;

// Mutexes
SemaphoreHandle_t xMutexOdom;
SemaphoreHandle_t xMutexImu;

odometry_t currentOdom;
imu_data_t currentImu;

void setup() {
  // Init hardware
  Serial.begin(115200);
  setupIMU();
  setupMotors();
  setupEncoders();

  // Init ROS
  nh.initNode();
  nh.advertise(odom_pub);
  nh.advertise(imu_pub);
  nh.subscribe(cmd_vel_sub);

  // Create Mutexes
  xMutexOdom = xSemaphoreCreateMutex();
  xMutexImu = xSemaphoreCreateMutex();

  // Create Tasks
  xTaskCreate(TaskMotorControl, "MotorCtrl", 256, NULL, 3, &xMotorControlTask);
  xTaskCreate(TaskImu, "IMU", 256, NULL, 2, &xImuTask);
  xTaskCreate(TaskOdometry, "Odom", 256, NULL, 2, &xOdometryTask);
  xTaskCreate(TaskRosCommunication, "ROSComm", 512, NULL, 1, &xRosCommunicationTask);

  vTaskStartScheduler();
}

void loop() {
  // Empty
}

// --- TASKS ---

// Motor Control Task
void TaskMotorControl(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 10; // 100Hz

  for (;;) {
    xTaskDelayUntil(&xLastWakeTime, xFrequency);
    motorControlLoopUpdate();
  }
}

// IMU Task
void TaskImu(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 20; // 50Hz

  for (;;) {
    xTaskDelayUntil(&xLastWakeTime, xFrequency);
    readAndFilterIMU();
  }
}

// Odometry Task
void TaskOdometry(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 20; // 50Hz

  for (;;) {
    xTaskDelayUntil(&xLastWakeTime, xFrequency);
    updateOdometry();
  }
}

// ROS Comm Task
void TaskRosCommunication(void *pvParameters) {
  for (;;) {
    nh.spinOnce();

    if (xSemaphoreTake(xMutexOdom, (TickType_t)10) == pdTRUE) {
      populateOdomMessage(currentOdom);
      xSemaphoreGive(xMutexOdom);
    }
    if (xSemaphoreTake(xMutexImu, (TickType_t)10) == pdTRUE) {
      populateImuMessage(currentImu);
      xSemaphoreGive(xMutexImu);
    }

    odom_pub.publish(&odom_msg);
    imu_pub.publish(&imu_msg);

    vTaskDelay(20);
  }
}

void cmdVelCallback(const geometry_msgs::Twist& msg) {
  // Handle cmd_vel
}
