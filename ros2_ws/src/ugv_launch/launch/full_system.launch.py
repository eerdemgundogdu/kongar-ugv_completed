from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Dirs
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    return LaunchDescription([
        # TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'laser']
        ),

        # LIDAR
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

        # Bridge
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
        
        # Lane
        Node(
            package='ugv_vision',
            executable='lane_detector',
            name='lane_detector',
            output='screen'
        ),
        
        # Object
        Node(
            package='ugv_vision',
            executable='object_detector',
            name='object_detector',
            output='screen'
        ),
        
        # Mission
        Node(
            package='ugv_control',
            executable='mission_controller',
            name='mission_controller',
            output='screen'
        ),
        
        # SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false'
            }.items()
        ),
        
        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
            }.items()
        ),
    ])
