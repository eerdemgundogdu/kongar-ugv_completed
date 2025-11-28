import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32, Bool, Float64MultiArray
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import time
import math

# States
STATE_INIT = 0
STATE_START = 1
STATE_SEGMENTATION = 2
STATE_SPIRAL_SEARCH = 3
STATE_GPS_NAVIGATION = 4
STATE_OBSTACLE_AVOID = 5
STATE_REPORT = 6

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        self.state = STATE_INIT
        self.initialized = False
        
        # Data
        self.lane_error = 0.0
        self.lane_found = False
        self.obstacle_detected = False
        self.current_pose = None
        self.gps_target_bearing = 0.0
        self.gps_target_distance = 0.0
        
        # Params
        self.spiral_start_time = 0
        self.spiral_radius = 0.5
        self.spiral_angle = 0.0
        
        # ROS
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        
        self.create_subscription(Float32, '/lane/error', self.lane_error_callback, 10)
        self.create_subscription(Bool, '/lane/found', self.lane_found_callback, 10)
        self.create_subscription(Bool, '/obstacle/detected', self.obstacle_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/visual_odom', self.visual_odom_callback, 10)
        self.create_subscription(Float64MultiArray, '/gps/target', self.gps_target_callback, 10)
        self.create_subscription(String, '/user_command', self.command_callback, 10)
        
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_goal_handle = None
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Mission Controller Active')

    def command_callback(self, msg):
        cmd = msg.data.upper()
        if cmd == 'INIT':
            self.get_logger().info('Initializing...')
            time.sleep(1)
            self.initialized = True
            self.state = STATE_START
            self.get_logger().info('Ready')
            
        elif cmd == 'START':
            if self.initialized:
                self.state = STATE_SEGMENTATION
                self.get_logger().info('Started')
            else:
                self.get_logger().warn('Not initialized')
                
        elif cmd == 'GPS_NAV':
            self.state = STATE_GPS_NAVIGATION
            self.get_logger().info('GPS Mode')
            self.send_nav2_goal()

    def lane_error_callback(self, msg):
        self.lane_error = msg.data

    def lane_found_callback(self, msg):
        self.lane_found = msg.data

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def visual_odom_callback(self, msg):
        if self.current_pose is None:
            self.current_pose = msg.pose.pose

    def gps_target_callback(self, msg):
        if len(msg.data) >= 2:
            self.gps_target_bearing = msg.data[0]
            self.gps_target_distance = msg.data[1]

    def spiral_search(self):
        elapsed = time.time() - self.spiral_start_time
        self.spiral_angle += 0.1
        self.spiral_radius = min(0.5 + elapsed * 0.1, 2.0)
        
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = 0.3
        return twist

    def send_nav2_goal(self):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 unavailable')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Placeholder
        target_x = 2.0
        target_y = 0.0

        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Nav to: ({target_x}, {target_y})')
        
        self._send_goal_future = self._nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._nav_goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Nav complete')
        self.state = STATE_REPORT

    def control_loop(self):
        twist = Twist()
        
        if self.state in [STATE_INIT, STATE_START, STATE_REPORT]:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        elif self.state == STATE_SEGMENTATION:
            if not self.lane_found:
                self.state = STATE_SPIRAL_SEARCH
                self.spiral_start_time = time.time()
                self.get_logger().warn('Lane lost, searching')
                return
            
            if self.obstacle_detected:
                self.state = STATE_OBSTACLE_AVOID
                self.get_logger().info('Obstacle!')
                return

            # Lane Follow
            kp = 0.015
            twist.linear.x = 0.25
            twist.angular.z = -kp * self.lane_error
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == STATE_SPIRAL_SEARCH:
            if self.lane_found:
                self.state = STATE_SEGMENTATION
                self.get_logger().info('Lane found')
                return
            
            twist = self.spiral_search()
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == STATE_GPS_NAVIGATION:
            # Nav2 handles this
            pass
            
        elif self.state == STATE_OBSTACLE_AVOID:
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            
            if not self.obstacle_detected:
                self.state = STATE_SEGMENTATION
                self.get_logger().info('Cleared')

        # Pub State
        state_names = {
            STATE_INIT: 'INIT',
            STATE_START: 'START',
            STATE_SEGMENTATION: 'SEGMENTATION',
            STATE_SPIRAL_SEARCH: 'SPIRAL_SEARCH',
            STATE_GPS_NAVIGATION: 'GPS_NAVIGATION',
            STATE_OBSTACLE_AVOID: 'OBSTACLE_AVOID',
            STATE_REPORT: 'REPORT'
        }
        state_msg = String()
        state_msg.data = state_names.get(self.state, 'UNKNOWN')
        self.state_pub.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
