import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class SlamIntegration(Node):
    def __init__(self):
        super().__init__('slam_integration')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.last_odom = None
        
        self.create_timer(0.02, self.publish_tf)
        
        self.get_logger().info('SLAM Integration Ready')
    
    def odom_callback(self, msg):
        self.last_odom = msg
    
    def scan_callback(self, msg):
        pass
    
    def publish_tf(self):
        if self.last_odom is None:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.last_odom.pose.pose.position.x
        t.transform.translation.y = self.last_odom.pose.pose.position.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation = self.last_odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = SlamIntegration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
