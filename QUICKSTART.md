# Quick Start Guide - Teknofest UGV

## Prerequisites Checklist
- [ ] ROS 2 Humble installed on Jetson Nano
- [ ] PlatformIO installed for Teensy
- [ ] Teensy 4.1 connected via USB
- [ ] Camera connected to Jetson
- [ ] Motors and ESCs connected to Teensy

## Installation

### 1. Install ROS 2 Dependencies
```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
pip3 install pyserial opencv-python
```

### 2. Build ROS 2 Workspace
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Flash Teensy Firmware
```bash
cd firmware
pio run -t upload
```

## Running the System

### Basic System (No SLAM/Nav2)
```bash
ros2 launch ugv_launch system.launch.py
```

### Full System (With SLAM and Navigation)
```bash
ros2 launch ugv_launch full_system.launch.py
```

## Control Commands

### Initialize System
```bash
ros2 topic pub /user_command std_msgs/String "data: 'INIT'" --once
```

### Start Mission
```bash
ros2 topic pub /user_command std_msgs/String "data: 'START'" --once
```

### Manual Control
```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Turn Left
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.3}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Monitoring

### View Mission State
```bash
ros2 topic echo /mission_state
```

### View Lane Error
```bash
ros2 topic echo /lane/error
```

### View Camera Feed
```bash
ros2 run rqt_image_view rqt_image_view /lane/image
```

## Troubleshooting

### Serial Port Not Found
```bash
ls /dev/ttyACM*
sudo usermod -a -G dialout $USER
# Logout and login again
```

### Motors Not Responding
- Check watchdog (should receive heartbeat every 100ms)
- Verify ESC calibration
- Check power supply

### Camera Not Working
```bash
ls /dev/video*
v4l2-ctl --list-devices
```

## Safety

**Emergency Stop**: Press Ctrl+C in the launch terminal or publish zero velocity:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

**Watchdog**: Motors automatically stop after 500ms without commands.
