from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ugv_launch_dir = get_package_share_directory('ugv_launch')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Custom config paths
    nav2_params = os.path.join(ugv_launch_dir, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(ugv_launch_dir, 'config', 'slam_params.yaml')
    
    return LaunchDescription([
        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['0.15', '0', '0.25', '0', '0', '0', 'base_link', 'camera_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'imu_link']
        ),

        # LIDAR Driver (sllidar_ros2)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0', 
                         'serial_baudrate': 115200, 
                         'frame_id': 'laser',
                         'inverted': False, 
                         'angle_compensate': True}],
            output='screen'
        ),
        
        # RPLidar Processor (obstacle detection & emergency stop)
        Node(
            package='ugv_vision',
            executable='rplidar_processor',
            name='rplidar_processor',
            parameters=[
                {'obstacle_distance': 0.5},
                {'emergency_distance': 0.25},
                {'front_angle_range': 60.0},
                {'enable_emergency_stop': True}
            ],
            output='screen'
        ),
        
        # LiDAR SLAM Bridge
        Node(
            package='ugv_vision',
            executable='lidar_slam_bridge',
            name='lidar_slam_bridge',
            output='screen'
        ),

        # Serial Bridge
        Node(
            package='ugv_interface',
            executable='serial_bridge',
            name='serial_bridge',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baudrate': 115200}
            ],
            output='screen'
        ),
        
        # Camera Driver
        Node(
            package='ugv_vision',
            executable='camera_driver',
            name='camera_driver',
            parameters=[
                {'device_id': 0},
                {'width': 640},
                {'height': 480},
                {'fps': 30}
            ],
            output='screen'
        ),
        
        # Lane Detector
        Node(
            package='ugv_vision',
            executable='lane_detector',
            name='lane_detector',
            output='screen'
        ),
        
        # Object Detector
        Node(
            package='ugv_vision',
            executable='object_detector',
            name='object_detector',
            output='screen'
        ),
        
        # Visual Odometry
        Node(
            package='ugv_vision',
            executable='visual_odometry',
            name='visual_odometry',
            output='screen'
        ),
        
        # SLAM Integration
        Node(
            package='ugv_vision',
            executable='slam_integration',
            name='slam_integration',
            output='screen'
        ),
        
        # GPS Navigator
        Node(
            package='ugv_control',
            executable='gps_navigator',
            name='gps_navigator',
            output='screen'
        ),
        
        # Mission Controller
        Node(
            package='ugv_control',
            executable='mission_controller',
            name='mission_controller',
            output='screen'
        ),
        
        # ROS2-Dashboard Bridge
        Node(
            package='ugv_interface',
            executable='ros2_bridge',
            name='ros2_bridge',
            parameters=[
                {'dashboard_url': 'http://localhost:5000'}
            ],
            output='screen'
        ),
        
        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': slam_params
            }.items()
        ),
        
        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': nav2_params
            }.items()
        ),
    ])

