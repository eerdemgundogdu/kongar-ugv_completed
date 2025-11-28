import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np

class SLAMIntegration(Node):
    def __init__(self):
        super().__init__('slam_integration')
        
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseStamped, '/slam_pose', self.pose_callback, 10)
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/ugv/map', 10)
        
        self.current_map = None
        self.current_pose = None
        
        self.get_logger().info('SLAM Integration Started')

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        
        if len(valid_ranges) > 0:
            min_dist = np.min(valid_ranges)
            if min_dist < 0.5:
                self.get_logger().warn(f'Obstacle detected at {min_dist:.2f}m')

    def map_callback(self, msg):
        self.current_map = msg
        self.map_pub.publish(msg)

    def pose_callback(self, msg):
        self.current_pose = msg
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'Position: ({x:.2f}, {y:.2f})', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = SLAMIntegration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
