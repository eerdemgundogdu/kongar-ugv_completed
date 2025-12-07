import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math

class LidarSlamBridge(Node):
    """
    Bridge node connecting RPLidar data with SLAM toolbox.
    Publishes TF transforms and processes map data.
    """
    
    def __init__(self):
        super().__init__('lidar_slam_bridge')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, reliable_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publishers
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/scan_filtered', sensor_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/slam_pose', 10)
        
        # State
        self.current_pose = None
        self.map_received = False
        self.scan_count = 0
        
        # Timer for TF publishing
        self.create_timer(0.02, self.publish_tf)  # 50Hz
        
        self.get_logger().info('LiDAR SLAM Bridge started')
    
    def scan_callback(self, msg):
        """Process and filter LiDAR scan data."""
        self.scan_count += 1
        
        # Filter out invalid readings
        ranges = list(msg.ranges)
        intensities = list(msg.intensities) if msg.intensities else []
        
        for i in range(len(ranges)):
            if not math.isfinite(ranges[i]):
                ranges[i] = msg.range_max
            elif ranges[i] < msg.range_min:
                ranges[i] = msg.range_min
        
        # Create filtered scan message
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = ranges
        filtered.intensities = intensities
        
        self.filtered_scan_pub.publish(filtered)
        
        # Log stats periodically
        if self.scan_count % 100 == 0:
            valid_count = sum(1 for r in ranges if msg.range_min < r < msg.range_max)
            self.get_logger().debug(
                f'Scan #{self.scan_count}: {valid_count}/{len(ranges)} valid readings'
            )
    
    def map_callback(self, msg):
        """Handle map updates from SLAM."""
        if not self.map_received:
            self.get_logger().info(
                f'Map received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/cell'
            )
            self.map_received = True
    
    def odom_callback(self, msg):
        """Track robot pose from odometry."""
        self.current_pose = msg.pose.pose
        
        # Publish as PoseStamped for SLAM
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'odom'
        pose_msg.pose = self.current_pose
        self.pose_pub.publish(pose_msg)
    
    def publish_tf(self):
        """Publish TF transforms for the sensor frame."""
        now = self.get_clock().now().to_msg()
        
        # map -> odom transform (identity if no SLAM correction)
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t_map_odom)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSlamBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
