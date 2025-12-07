import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import math

class GPSNavigator(Node):
    def __init__(self):
        super().__init__('gps_navigator')
        
        self.current_lat = None
        self.current_lon = None
        
        self.waypoints = []
        self.current_waypoint_idx = 0
        
        self.target_pub = self.create_publisher(Float64MultiArray, '/gps/target', 10)
        self.distance_pub = self.create_publisher(Float64MultiArray, '/gps/distance', 10)
        
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Float64MultiArray, '/gps/waypoints', self.waypoints_callback, 10)
        
        self.create_timer(0.5, self.navigation_loop)
        
        self.get_logger().info('Started')

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def waypoints_callback(self, msg):
        self.waypoints = []
        for i in range(0, len(msg.data), 2):
            if i + 1 < len(msg.data):
                self.waypoints.append((msg.data[i], msg.data[i+1]))
        self.current_waypoint_idx = 0
        self.get_logger().info(f'Loaded {len(self.waypoints)} pts')

    def haversine_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)
        
        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360

    def navigation_loop(self):
        if self.current_lat is None or not self.waypoints:
            return
        
        if self.current_waypoint_idx >= len(self.waypoints):
            self.get_logger().info('Done')
            return
        
        target_lat, target_lon = self.waypoints[self.current_waypoint_idx]
        
        distance = self.haversine_distance(
            self.current_lat, self.current_lon,
            target_lat, target_lon
        )
        
        bearing = self.calculate_bearing(
            self.current_lat, self.current_lon,
            target_lat, target_lon
        )
        
        target_msg = Float64MultiArray()
        target_msg.data = [bearing, distance]
        self.target_pub.publish(target_msg)
        
        if distance < 2.0:
            self.get_logger().info(f'Pt {self.current_waypoint_idx} reached')
            self.current_waypoint_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
