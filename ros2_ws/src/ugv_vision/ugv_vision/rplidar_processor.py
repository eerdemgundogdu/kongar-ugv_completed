import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
import numpy as np
import math

class RPLidarProcessor(Node):
    """
    RPLidar processor node for obstacle detection and safety monitoring.
    Works with RPLidar A1/A2/A3 via sllidar_ros2 package.
    """
    
    def __init__(self):
        super().__init__('rplidar_processor')
        
        # Parameters
        self.declare_parameter('obstacle_distance', 0.5)  # meters
        self.declare_parameter('emergency_distance', 0.25)  # meters
        self.declare_parameter('front_angle_range', 60.0)  # degrees
        self.declare_parameter('side_angle_range', 30.0)  # degrees
        self.declare_parameter('enable_emergency_stop', True)
        
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.emergency_distance = self.get_parameter('emergency_distance').value
        self.front_angle = self.get_parameter('front_angle_range').value
        self.side_angle = self.get_parameter('side_angle_range').value
        self.emergency_stop_enabled = self.get_parameter('enable_emergency_stop').value
        
        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        
        # Publishers
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle/detected', 10)
        self.obstacle_front_pub = self.create_publisher(Bool, '/obstacle/front', 10)
        self.obstacle_left_pub = self.create_publisher(Bool, '/obstacle/left', 10)
        self.obstacle_right_pub = self.create_publisher(Bool, '/obstacle/right', 10)
        self.min_distance_pub = self.create_publisher(Float32, '/obstacle/min_distance', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.safe_cmd_vel_pub = self.create_publisher(Twist, '/safe_cmd_vel', 10)
        
        # Subscribe to cmd_vel for safety filtering
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # State
        self.current_cmd_vel = Twist()
        self.emergency_active = False
        self.last_scan_time = self.get_clock().now()
        
        # Timer for watchdog
        self.create_timer(0.5, self.watchdog_callback)
        
        self.get_logger().info(f'RPLidar Processor started')
        self.get_logger().info(f'  Obstacle distance: {self.obstacle_distance}m')
        self.get_logger().info(f'  Emergency distance: {self.emergency_distance}m')
        self.get_logger().info(f'  Front angle: ±{self.front_angle/2}°')
    
    def cmd_vel_callback(self, msg):
        self.current_cmd_vel = msg
    
    def watchdog_callback(self):
        """Check if we're receiving scan data."""
        elapsed = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if elapsed > 2.0:
            self.get_logger().warn('No LiDAR data received for 2s', throttle_duration_sec=5.0)
    
    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Replace inf/nan with max range
        ranges = np.where(np.isfinite(ranges), ranges, msg.range_max)
        
        # Calculate angles for each reading
        angles_rad = angle_min + np.arange(len(ranges)) * angle_increment
        angles_deg = np.degrees(angles_rad)
        
        # Normalize angles to [-180, 180]
        angles_deg = ((angles_deg + 180) % 360) - 180
        
        # Define zones
        front_mask = np.abs(angles_deg) < (self.front_angle / 2)
        left_mask = (angles_deg > 60) & (angles_deg < 120)
        right_mask = (angles_deg < -60) & (angles_deg > -120)
        
        # Get minimum distances per zone
        front_min = np.min(ranges[front_mask]) if np.any(front_mask) else msg.range_max
        left_min = np.min(ranges[left_mask]) if np.any(left_mask) else msg.range_max
        right_min = np.min(ranges[right_mask]) if np.any(right_mask) else msg.range_max
        overall_min = np.min(ranges)
        
        # Obstacle detection
        front_obstacle = front_min < self.obstacle_distance
        left_obstacle = left_min < self.obstacle_distance
        right_obstacle = right_min < self.obstacle_distance
        any_obstacle = front_obstacle or left_obstacle or right_obstacle
        
        # Emergency stop check
        emergency = front_min < self.emergency_distance
        
        # Publish obstacle states
        self.obstacle_pub.publish(Bool(data=any_obstacle))
        self.obstacle_front_pub.publish(Bool(data=front_obstacle))
        self.obstacle_left_pub.publish(Bool(data=left_obstacle))
        self.obstacle_right_pub.publish(Bool(data=right_obstacle))
        self.min_distance_pub.publish(Float32(data=float(overall_min)))
        self.emergency_stop_pub.publish(Bool(data=emergency))
        
        # Safety-filtered cmd_vel
        safe_cmd = Twist()
        if emergency and self.emergency_stop_enabled:
            # Emergency stop - zero velocity
            if not self.emergency_active:
                self.get_logger().warn(f'EMERGENCY STOP! Object at {front_min:.2f}m')
                self.emergency_active = True
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = self.current_cmd_vel.angular.z  # Allow turning
        elif front_obstacle and self.current_cmd_vel.linear.x > 0:
            # Slow down when obstacle ahead
            if self.emergency_active:
                self.get_logger().info('Emergency cleared')
                self.emergency_active = False
            safe_cmd.linear.x = min(self.current_cmd_vel.linear.x, 0.1)
            safe_cmd.angular.z = self.current_cmd_vel.angular.z
        else:
            if self.emergency_active:
                self.get_logger().info('Path clear')
                self.emergency_active = False
            safe_cmd = self.current_cmd_vel
        
        self.safe_cmd_vel_pub.publish(safe_cmd)
        
        # Debug logging
        if any_obstacle:
            self.get_logger().debug(
                f'Obstacles - Front: {front_min:.2f}m, Left: {left_min:.2f}m, Right: {right_min:.2f}m',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = RPLidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
