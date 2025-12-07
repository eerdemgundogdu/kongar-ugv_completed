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
STATE_EMERGENCY_STOP = 6
STATE_REPORT = 7

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_kp', 0.015)
        self.declare_parameter('obstacle_turn_speed', 0.5)
        self.declare_parameter('spiral_search_timeout', 30.0)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.obstacle_turn_speed = self.get_parameter('obstacle_turn_speed').value
        self.spiral_timeout = self.get_parameter('spiral_search_timeout').value
        
        self.state = STATE_INIT
        self.prev_state = STATE_INIT
        self.initialized = False
        
        # Sensor Data
        self.lane_error = 0.0
        self.lane_found = False
        self.obstacle_detected = False
        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.emergency_stop = False
        self.min_distance = 10.0
        self.battery_level = 100.0
        self.current_pose = None
        self.gps_target_bearing = 0.0
        self.gps_target_distance = 0.0
        
        # Spiral search
        self.spiral_start_time = 0
        self.spiral_angle = 0.0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        
        # Subscriptions
        self.create_subscription(Float32, '/lane/error', self.lane_error_callback, 10)
        self.create_subscription(Bool, '/lane/found', self.lane_found_callback, 10)
        self.create_subscription(Bool, '/obstacle/detected', self.obstacle_callback, 10)
        self.create_subscription(Bool, '/obstacle/front', self.obstacle_front_callback, 10)
        self.create_subscription(Bool, '/obstacle/left', self.obstacle_left_callback, 10)
        self.create_subscription(Bool, '/obstacle/right', self.obstacle_right_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)
        self.create_subscription(Float32, '/obstacle/min_distance', self.min_distance_callback, 10)
        self.create_subscription(Float32, '/battery/state', self.battery_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/visual_odom', self.visual_odom_callback, 10)
        self.create_subscription(Float64MultiArray, '/gps/target', self.gps_target_callback, 10)
        self.create_subscription(String, '/user_command', self.command_callback, 10)
        
        # Nav2 Action Client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_goal_handle = None
        
        # Control Timer
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Mission Controller Active')

    # Callbacks
    def command_callback(self, msg):
        cmd = msg.data.upper().strip()
        
        if cmd == 'INIT':
            self.get_logger().info('Initializing...')
            time.sleep(1)
            self.initialized = True
            self.state = STATE_START
            self.get_logger().info('Ready')
            
        elif cmd == 'START':
            if self.initialized:
                self.state = STATE_SEGMENTATION
                self.get_logger().info('Mission Started')
            else:
                self.get_logger().warn('Not initialized! Send INIT first.')
                
        elif cmd == 'STOP':
            self.state = STATE_INIT
            self.stop_robot()
            self.get_logger().info('Mission Stopped')
                
        elif cmd == 'GPS_NAV':
            self.state = STATE_GPS_NAVIGATION
            self.get_logger().info('GPS Navigation Mode')
            self.send_nav2_goal()
            
        elif cmd == 'REPORT':
            self.state = STATE_REPORT
            self.stop_robot()
            self.get_logger().info('Mission Complete - Report')

    def lane_error_callback(self, msg):
        self.lane_error = msg.data

    def lane_found_callback(self, msg):
        self.lane_found = msg.data

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        
    def obstacle_front_callback(self, msg):
        self.obstacle_front = msg.data
        
    def obstacle_left_callback(self, msg):
        self.obstacle_left = msg.data
        
    def obstacle_right_callback(self, msg):
        self.obstacle_right = msg.data
        
    def emergency_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop and self.state != STATE_EMERGENCY_STOP:
            self.prev_state = self.state
            self.state = STATE_EMERGENCY_STOP
            self.get_logger().warn('EMERGENCY STOP TRIGGERED!')
            
    def min_distance_callback(self, msg):
        self.min_distance = msg.data
        
    def battery_callback(self, msg):
        self.battery_level = msg.data
        if self.battery_level < 20.0:
            self.get_logger().warn(f'Low battery: {self.battery_level:.1f}%', 
                                   throttle_duration_sec=30.0)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def visual_odom_callback(self, msg):
        if self.current_pose is None:
            self.current_pose = msg.pose.pose

    def gps_target_callback(self, msg):
        if len(msg.data) >= 2:
            self.gps_target_bearing = msg.data[0]
            self.gps_target_distance = msg.data[1]

    # Control Methods
    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def spiral_search(self):
        elapsed = time.time() - self.spiral_start_time
        self.spiral_angle += 0.1
        
        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = max(0.2, 0.5 - elapsed * 0.01)
        return twist

    def avoid_obstacle(self):
        twist = Twist()
        twist.linear.x = 0.0
        
        if self.obstacle_left and not self.obstacle_right:
            twist.angular.z = -self.obstacle_turn_speed  # Turn right
        elif self.obstacle_right and not self.obstacle_left:
            twist.angular.z = self.obstacle_turn_speed   # Turn left
        else:
            twist.angular.z = self.obstacle_turn_speed   # Default left
            
        return twist

    def send_nav2_goal(self):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 server unavailable')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = 2.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Sending Nav2 goal')
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
        self.get_logger().info('Navigation complete')
        self.state = STATE_REPORT

    # Main Control Loop
    def control_loop(self):
        twist = Twist()
        
        # Emergency stop takes priority
        if self.state == STATE_EMERGENCY_STOP:
            self.stop_robot()
            if not self.emergency_stop:
                self.state = self.prev_state
                self.get_logger().info('Emergency cleared, resuming')
            self.publish_state()
            return
        
        if self.state in [STATE_INIT, STATE_START, STATE_REPORT]:
            self.stop_robot()
            
        elif self.state == STATE_SEGMENTATION:
            if self.obstacle_front:
                self.state = STATE_OBSTACLE_AVOID
                self.get_logger().info('Obstacle ahead, avoiding')
                return
                
            if not self.lane_found:
                self.state = STATE_SPIRAL_SEARCH
                self.spiral_start_time = time.time()
                self.get_logger().warn('Lane lost, searching')
                return

            # Lane following with P control
            twist.linear.x = self.linear_speed
            twist.angular.z = -self.angular_kp * self.lane_error
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == STATE_SPIRAL_SEARCH:
            if self.lane_found:
                self.state = STATE_SEGMENTATION
                self.get_logger().info('Lane found')
                return
                
            elapsed = time.time() - self.spiral_start_time
            if elapsed > self.spiral_timeout:
                self.get_logger().warn('Spiral search timeout, switching to GPS')
                self.state = STATE_GPS_NAVIGATION
                return
            
            twist = self.spiral_search()
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == STATE_GPS_NAVIGATION:
            pass  # Nav2 handles this
            
        elif self.state == STATE_OBSTACLE_AVOID:
            twist = self.avoid_obstacle()
            self.cmd_vel_pub.publish(twist)
            
            if not self.obstacle_front:
                self.state = STATE_SEGMENTATION
                self.get_logger().info('Path cleared')

        self.publish_state()
        
    def publish_state(self):
        state_names = {
            STATE_INIT: 'INIT',
            STATE_START: 'START',
            STATE_SEGMENTATION: 'SEGMENTATION',
            STATE_SPIRAL_SEARCH: 'SPIRAL_SEARCH',
            STATE_GPS_NAVIGATION: 'GPS_NAVIGATION',
            STATE_OBSTACLE_AVOID: 'OBSTACLE_AVOID',
            STATE_EMERGENCY_STOP: 'EMERGENCY_STOP',
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
