# Competition Requirements Checklist

Based on the flowchart analysis, here's what you have for Teknofest:

## ✅ Implemented Components

### Core System
- [x] **INIT State**: Sensor initialization logic
- [x] **START State**: Mission start trigger
- [x] **SEGMENTATION State**: Lane following with vision
- [x] **SPIRAL SEARCH**: Automatic spiral pattern when line is lost
- [x] **GPS Navigation**: Waypoint navigation with haversine distance calculation
- [x] **Obstacle Avoidance**: Detection and avoidance logic
- [x] **Hardware Watchdog**: 500ms timeout on Teensy
- [x] **Serial Communication**: ROS 2 ↔ Teensy via UART

### ROS 2 Packages
- [x] `ugv_interface`: Serial bridge with heartbeat
- [x] `ugv_control`: Mission controller + GPS navigator
- [x] `ugv_vision`: Lane detector + Object detector
- [x] `ugv_launch`: System launch files

### Teensy Firmware
- [x] FreeRTOS with 3 tasks (Motor 100Hz, Sensors 50Hz, Comms)
- [x] Motor driver with FOC support structure
- [x] PID controller
- [x] Watchdog safety system

## 📋 What You Need to Do Before Competition

### Hardware Setup
1. **Connect GPS Module** to Jetson (usually `/dev/ttyUSB0` or `/dev/ttyACM1`)
2. **Calibrate Camera** - Adjust HSV thresholds in `lane_detector.py` for your track
3. **Tune PID** - Adjust `Kp`, `Ki`, `Kd` in `firmware/include/config.h`
4. **Test Motors** - Verify ESC calibration and direction

### Software Configuration
1. **Set Serial Port** in `system.launch.py` (default: `/dev/ttyACM0`)
2. **Load GPS Waypoints**:
   ```bash
   ros2 topic pub /gps/waypoints std_msgs/Float64MultiArray \
     "data: [39.123456, 32.654321, 39.234567, 32.765432]"
   ```
3. **Adjust Lane Detection** - Test with your track and tune color thresholds

### Testing Sequence
1. **Motor Test**: `cd firmware && pio run -t upload && pio device monitor`
2. **Vision Test**: `ros2 run ugv_vision lane_detector` (check `/lane/image` topic)
3. **Full System**: `ros2 launch ugv_launch full_system.launch.py`
4. **Competition Run**:
   ```bash
   ros2 topic pub /user_command std_msgs/String "data: 'INIT'" --once
   ros2 topic pub /user_command std_msgs/String "data: 'START'" --once
   ```

## 🎯 Competition Features

| Feature | Status | Notes |
|---------|--------|-------|
| Lane Following | ✅ Ready | Tune HSV thresholds for your track |
| Spiral Search | ✅ Ready | Activates when line lost >500ms |
| GPS Navigation | ✅ Ready | Load waypoints before run |
| Obstacle Avoidance | ✅ Ready | Uses color detection (tune for competition) |
| SLAM Mapping | ✅ Ready | Launch with `full_system.launch.py` |
| A* Path Planning | ✅ Ready | Via Nav2 integration |
| Safety Watchdog | ✅ Ready | 500ms timeout |

## 📁 Key Files to Review

1. **Mission Logic**: `ros2_ws/src/ugv_control/ugv_control/mission_controller.py`
2. **Lane Detection**: `ros2_ws/src/ugv_vision/ugv_vision/lane_detector.py`
3. **GPS Navigation**: `ros2_ws/src/ugv_control/ugv_control/gps_navigator.py`
4. **Firmware**: `firmware/src/main.cpp`
5. **Launch**: `ros2_ws/src/ugv_launch/launch/full_system.launch.py`

## 🚀 Quick Start Commands

```bash
# Build
cd ros2_ws && colcon build --symlink-install && source install/setup.bash

# Flash Teensy
cd ../firmware && pio run -t upload

# Run System
ros2 launch ugv_launch full_system.launch.py

# Initialize and Start
ros2 topic pub /user_command std_msgs/String "data: 'INIT'" --once
ros2 topic pub /user_command std_msgs/String "data: 'START'" --once
```

## ⚠️ Important Notes

1. **Spiral Search**: Automatically activates when lane is lost (contour area < 500 pixels)
2. **GPS Waypoints**: Must be loaded via `/gps/waypoints` topic before navigation
3. **Watchdog**: Teensy stops motors if no command for 500ms (safety feature)
4. **State Transitions**: INIT → START → SEGMENTATION ↔ SPIRAL_SEARCH ↔ GPS_NAV

Good luck at Teknofest! 🏆
