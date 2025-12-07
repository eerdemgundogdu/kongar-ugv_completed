import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import socketio
import threading
import math

class ROS2DashboardBridge(Node):
    def __init__(self):
        super().__init__('ros2_dashboard_bridge')
        
        self.declare_parameter('dashboard_url', 'http://localhost:5000')
        dashboard_url = self.get_parameter('dashboard_url').value
        
        self.sio = socketio.Client()
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.user_cmd_pub = self.create_publisher(String, '/user_command', 10)
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(String, '/mission_state', self.mission_state_callback, 10)
        
        @self.sio.on('cmd_vel_to_ros')
        def on_cmd_vel(data):
            msg = Twist()
            msg.linear.x = float(data.get('linear', 0.0))
            msg.angular.z = float(data.get('angular', 0.0))
            self.cmd_vel_pub.publish(msg)
        
        @self.sio.on('user_command_to_ros')
        def on_user_command(data):
            msg = String()
            msg.data = data.get('command', '')
            self.user_cmd_pub.publish(msg)
        
        @self.sio.on('connect')
        def on_connect():
            self.get_logger().info('Connected to dashboard')
        
        @self.sio.on('disconnect')
        def on_disconnect():
            self.get_logger().warn('Disconnected from dashboard')
        
        self.connect_thread = threading.Thread(target=self.connect_to_dashboard, args=(dashboard_url,), daemon=True)
        self.connect_thread.start()
        
        self.get_logger().info(f'ROS2 Dashboard Bridge started, connecting to {dashboard_url}')
    
    def connect_to_dashboard(self, url):
        while True:
            try:
                if not self.sio.connected:
                    self.sio.connect(url)
                    break
            except Exception as e:
                self.get_logger().warn(f'Connection failed: {e}, retrying in 5s...')
                import time
                time.sleep(5)
    
    def odom_callback(self, msg):
        if self.sio.connected:
            theta = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
            self.sio.emit('telemetry_update', {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'theta': theta,
                'linear_vel': msg.twist.twist.linear.x,
                'angular_vel': msg.twist.twist.angular.z
            })
    
    def imu_callback(self, msg):
        if self.sio.connected:
            self.sio.emit('imu_update', {
                'ax': msg.linear_acceleration.x,
                'ay': msg.linear_acceleration.y,
                'az': msg.linear_acceleration.z,
                'gx': msg.angular_velocity.x,
                'gy': msg.angular_velocity.y,
                'gz': msg.angular_velocity.z
            })
    
    def mission_state_callback(self, msg):
        if self.sio.connected:
            self.sio.emit('mission_state_update', {'state': msg.data})
    
    def destroy_node(self):
        if self.sio.connected:
            self.sio.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ROS2DashboardBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
