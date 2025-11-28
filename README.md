# Teknofest UGV Project 🏆

> Autonomous Unmanned Ground Vehicle for Teknofest Competition

## 🌟 Features

- **Advanced Vision**: YOLOv8 object detection + OpenCV lane following
- **GPS-less Navigation**: Visual Odometry for camera-based localization
- **SLAM & Path Planning**: slam_toolbox + Nav2 integration
- **Real-time Control**: FreeRTOS-based motor control with hardware watchdog
- **Modular Architecture**: Clean ROS 2 + Teensy design

## 🏗️ Architecture

### High-Level Computer (NVIDIA Jetson Nano)
**ROS 2 Humble** running Python nodes:
- `ugv_interface` - Serial bridge to Teensy
- `ugv_control` - Mission state machine + GPS navigation
- `ugv_vision` - Lane detection, object detection, visual odometry, SLAM
- `ugv_launch` - System orchestration

### Low-Level Controller (Teensy 4.1)
**FreeRTOS** running C++ firmware:
- Motor control (100Hz PID loop)
- Sensor reading (IMU, encoders at 50Hz)
- Serial communication with Jetson
- Hardware watchdog (500ms timeout)

## 🚀 Quick Start

### Prerequisites

**Jetson Nano:**
```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop ros-humble-slam-toolbox \
  ros-humble-navigation2 ros-humble-nav2-bringup

# Python packages
pip3 install -r requirements.txt
```

**Teensy 4.1:**
- Install [PlatformIO](https://platformio.org/)

### Build & Run

```bash
# 1. Build ROS 2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# 2. Flash Teensy firmware
cd ../firmware
pio run -t upload

# 3. Launch system
ros2 launch ugv_launch full_system.launch.py
```

### Control Commands

```bash
# Initialize
ros2 topic pub /user_command std_msgs/String "data: 'INIT'" --once

# Start mission
ros2 topic pub /user_command std_msgs/String "data: 'START'" --once

# Manual control
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

## 🤖 Advanced Features

### Custom YOLOv8 Training
Train object detection for parkour obstacles:
```bash
cd training
python train_yolo.py
```
See [`training/README.md`](training/README.md) for details.

### Visual Odometry (GPS-less)
Camera-based localization without GPS:
```bash
ros2 run ugv_vision visual_odometry
```
See [`ADVANCED_FEATURES.md`](ADVANCED_FEATURES.md) for full guide.

## 📁 Project Structure

```
UGV-main/
├── ros2_ws/src/          # ROS 2 packages
│   ├── ugv_interface/    # Serial communication
│   ├── ugv_control/      # Mission logic
│   ├── ugv_vision/       # Vision & SLAM
│   └── ugv_launch/       # Launch files
├── firmware/             # Teensy 4.1 code
│   ├── src/              # FreeRTOS tasks
│   └── include/          # Headers
├── training/             # YOLOv8 training
├── Lidar/                # SLAM integration
└── Stand-Alone-Bare-Metal-main/  # Alternative firmware
```

## 🎯 Mission States

1. **INIT** - System initialization
2. **START** - Ready to begin
3. **SEGMENTATION** - Lane following mode
4. **SPIRAL_SEARCH** - Search for lost lane
5. **GPS_NAVIGATION** - Navigate to waypoints
6. **OBSTACLE_AVOID** - Avoid detected obstacles
7. **REPORT** - Mission complete

## 🛡️ Safety Features

- Hardware watchdog (motors stop if no command for 500ms)
- Emergency stop capability
- Mutex-protected FreeRTOS tasks
- Graceful degradation (Visual Odometry fallback)

## 📚 Documentation

- [`QUICKSTART.md`](QUICKSTART.md) - Step-by-step setup guide
- [`ADVANCED_FEATURES.md`](ADVANCED_FEATURES.md) - ML features guide
- [`COMPETITION_CHECKLIST.md`](COMPETITION_CHECKLIST.md) - Pre-race checklist
- [`Stand-Alone-Bare-Metal-main/TESTING_GUIDE.md`](Stand-Alone-Bare-Metal-main/TESTING_GUIDE.md) - Bare-metal testing

## 🔧 Troubleshooting

### Serial Connection
```bash
ls /dev/ttyACM*
sudo usermod -a -G dialout $USER  # Then logout/login
```

### Motors Not Responding
- Check watchdog (heartbeat every 100ms)
- Verify ESC calibration
- Check power supply

### Camera Issues
```bash
ls /dev/video*
v4l2-ctl --list-devices
```

## 👥 Team

- **Emirhan Oğul**
- **Yusuf Ziya Bakırkol**
- **Erdem Gündoğdu**

## 📄 License

MIT License - see LICENSE file for details

---

**Built for Teknofest 2025** 🇹🇷
