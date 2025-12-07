#!/usr/bin/env python3
"""
Simple verification script to test UGV system integration.
Publishes cmd_vel and checks for system response.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class SystemVerifier(Node):
    def __init__(self):
        super().__init__('system_verifier')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.user_cmd_pub = self.create_publisher(String, '/user_command', 10)
        
        self.create_subscription(String, '/mission_state', self.state_callback, 10)
        
        self.current_state = None
        self.get_logger().info('System Verifier Started')

    def state_callback(self, msg):
        self.current_state = msg.data
        self.get_logger().info(f'Mission State: {self.current_state}')

    def run_tests(self):
        time.sleep(2)  # Wait for system to start
        
        # Test 1: Initialize
        self.get_logger().info('Test 1: Sending INIT command')
        init_msg = String()
        init_msg.data = 'INIT'
        self.user_cmd_pub.publish(init_msg)
        time.sleep(2)
        
        # Test 2: Start
        self.get_logger().info('Test 2: Sending START command')
        start_msg = String()
        start_msg.data = 'START'
        self.user_cmd_pub.publish(start_msg)
        time.sleep(2)
        
        # Test 3: Send velocity command
        self.get_logger().info('Test 3: Sending velocity command (forward)')
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(3)
        
        # Test 4: Turn
        self.get_logger().info('Test 4: Sending turn command')
        twist.linear.x = 0.1
        twist.angular.z = 0.3
        self.cmd_vel_pub.publish(twist)
        time.sleep(3)
        
        # Test 5: Stop
        self.get_logger().info('Test 5: Stopping')
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(2)
        
        self.get_logger().info('Verification tests completed!')

def main(args=None):
    rclpy.init(args=args)
    verifier = SystemVerifier()
    
    # Run tests in a separate thread
    import threading
    test_thread = threading.Thread(target=verifier.run_tests)
    test_thread.start()
    
    rclpy.spin(verifier)
    
    test_thread.join()
    verifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
