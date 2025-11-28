# SLAM Integration for UGV

This folder contains SLAM (Simultaneous Localization and Mapping) integration for the UGV.

## Components

### SLAM.py
ROS 2 node that integrates with `slam_toolbox` to:
- Process LiDAR data from `/scan` topic
- Monitor SLAM-generated map from `/map` topic
- Track robot pose from `/slam_pose` topic
- Detect obstacles from LiDAR readings

## Usage

### With slam_toolbox (Recommended)

The system uses ROS 2's `slam_toolbox` for SLAM. Launch it with:

```bash
ros2 launch ugv_launch full_system.launch.py
```

This automatically starts:
- `slam_toolbox` for mapping
- `SLAM.py` for integration
- Navigation stack (Nav2)

### Manual SLAM Launch

```bash
# Terminal 1: SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# Terminal 2: SLAM Integration
ros2 run ugv_vision slam_integration

# Terminal 3: Visualize
rviz2
```

## LiDAR Configuration

Supported LiDAR sensors:
- RPLidar A1/A2/A3
- YDLIDAR X2/X4
- Hokuyo URG-04LX

Configure in launch file:
```python
Node(
    package='rplidar_ros',
    executable='rplidar_composition',
    parameters=[{'serial_port': '/dev/ttyUSB0'}]
)
```

## Map Saving

Save the generated map:
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

This creates:
- `my_map.pgm` - Map image
- `my_map.yaml` - Map metadata
