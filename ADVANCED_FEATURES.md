# Advanced AI Features for Teknofest UGV

## 🎯 Overview
This document describes the advanced machine learning features implemented for the Teknofest competition, replacing the legacy scripts with modern ROS 2 + ML architecture.

## 🤖 Custom YOLOv8 Object Detection

### What It Does
- Detects parkour obstacles (ramps, traffic signs, parking spots, pedestrians)
- Trained specifically for Teknofest competition environment
- Replaces legacy `Advanced OD.py` and `advanced_OD.py`

### Files
- **Training Script**: `training/train_yolo.py`
- **Dataset Config**: `training/teknofest_parkour.yaml`
- **ROS 2 Node**: `ros2_ws/src/ugv_vision/ugv_vision/object_detector.py`

### How to Use

#### 1. Collect Training Data
```bash
# Take photos of obstacles during practice runs
# Organize in: training/datasets/teknofest_parkour/images/train/
```

#### 2. Label Your Data
Use [Roboflow](https://roboflow.com/) or [LabelImg](https://github.com/heartexlabs/labelImg):
- Label obstacles, ramps, traffic signs, etc.
- Export in YOLO format

#### 3. Train the Model
```bash
cd training
python train_yolo.py
```

#### 4. Deploy
```bash
# Copy trained model to vision package
cp runs/detect/teknofest_model/weights/best.pt ../ros2_ws/src/ugv_vision/models/

# Update object_detector.py to use your model:
# Change line 26: self.model = YOLO('models/best.pt')
```

## 📍 Visual Odometry (GPS-less Localization)

### What It Does
- Estimates robot position using **camera only** (no GPS needed!)
- Uses Optical Flow (Lucas-Kanade) to track movement
- Replaces legacy `GPS.py` with ML-based approach

### Files
- **ROS 2 Node**: `ros2_ws/src/ugv_vision/ugv_vision/visual_odometry.py`

### How It Works
1. Detects features in camera frames (ORB keypoints)
2. Tracks features between frames (Optical Flow)
3. Estimates camera motion (Essential Matrix + Pose Recovery)
4. Publishes position to `/visual_odom`

### Integration
The `mission_controller.py` automatically uses Visual Odometry as a fallback:
```python
def visual_odom_callback(self, msg):
    if self.current_pose is None:  # If wheel odom fails
        self.current_pose = msg.pose.pose  # Use camera-based position
```

### Camera Calibration (Important!)
Visual Odometry requires camera calibration. Update the camera matrix in `visual_odometry.py`:

```python
# Line 25-27: Replace with YOUR camera's calibration
self.K = np.array([[fx, 0, cx],
                   [0, fy, cy],
                   [0, 0, 1]], dtype=np.float32)
```

To calibrate:
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw
```

## 🚀 Running the System

### Basic (Lane Following + Object Detection)
```bash
ros2 launch ugv_launch system.launch.py
```

### With Visual Odometry
```bash
# Terminal 1: Launch base system
ros2 launch ugv_launch system.launch.py

# Terminal 2: Start Visual Odometry
ros2 run ugv_vision visual_odometry
```

### Full System (SLAM + Nav2 + Visual Odom)
```bash
ros2 launch ugv_launch full_system.launch.py
```

## 📊 Monitoring

### Check Visual Odometry
```bash
ros2 topic echo /visual_odom
```

### Check Object Detection
```bash
ros2 topic echo /obstacle/detected
```

### Visualize in RViz
```bash
rviz2
# Add: /visual_odom (Odometry), /camera/image_raw (Image)
```

## 🔧 Tuning Parameters

### Visual Odometry
Edit `visual_odometry.py`:
- **Line 48**: Feature detector (ORB, FAST, SIFT)
- **Line 61**: Optical Flow parameters
- **Line 69**: RANSAC threshold for outlier rejection

### YOLOv8
Edit `train_yolo.py`:
- **epochs**: More = better accuracy (but slower training)
- **imgsz**: Image size (640 is good for Jetson Nano)
- **batch**: Reduce if out of memory

## 📝 Legacy Files Reference

The following legacy files have been **replaced** by the new ROS 2 architecture:

| Legacy File | Replaced By | Notes |
|-------------|-------------|-------|
| `Advanced OD.py` | `ugv_vision/object_detector.py` | Now uses YOLOv8 |
| `advanced_OD.py` | `ugv_vision/object_detector.py` | Same as above |
| `GPS.py` | `ugv_vision/visual_odometry.py` | Camera-based localization |
| `Image+MotorControl.py` | `mission_controller.py` + `lane_detector.py` | Separated concerns |
| `MotorPID.py` | `firmware/src/pid_controller.cpp` | Moved to Teensy |
| `PIDforOD.py` | `mission_controller.py` | Integrated into state machine |

## 🎓 Learning Resources

- **YOLOv8 Docs**: https://docs.ultralytics.com/
- **Visual Odometry**: https://en.wikipedia.org/wiki/Visual_odometry
- **ROS 2 Navigation**: https://navigation.ros.org/

## ⚠️ Important Notes

1. **Visual Odometry has scale ambiguity** - it doesn't know absolute distances. Fuse with wheel odometry for best results.
2. **YOLOv8 needs GPU** - Training on CPU is very slow. Use Google Colab if you don't have a GPU.
3. **Camera calibration is critical** - Poor calibration = poor Visual Odometry performance.

---
**Ready for Teknofest!** 🏆
