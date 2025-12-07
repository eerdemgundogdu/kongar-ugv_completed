from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
    ])
