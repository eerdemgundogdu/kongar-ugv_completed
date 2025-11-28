import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        
        self.bridge = CvBridge()
        
        self.lane_error_pub = self.create_publisher(Float32, '/lane/error', 10)
        self.lane_found_pub = self.create_publisher(Bool, '/lane/found', 10)
        self.debug_image_pub = self.create_publisher(Image, '/lane/image', 10)
        
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        self.get_logger().info('Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Bridge Error: {e}')
            return

        height, width, _ = cv_image.shape
        
        roi = cv_image[int(height/2):, :]
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        error = 0.0
        lane_found = False
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 500:
                lane_found = True
                
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    cv2.circle(roi, (cx, cy), 10, (0, 255, 0), -1)
                    
                    center_x = width / 2
                    error = (cx - center_x) / center_x
                    
                    cv2.line(roi, (int(center_x), 0), (int(center_x), height), (255, 0, 0), 2)

        error_msg = Float32()
        error_msg.data = float(error)
        self.lane_error_pub.publish(error_msg)
        
        found_msg = Bool()
        found_msg.data = lane_found
        self.lane_found_pub.publish(found_msg)
        
        debug_msg = self.bridge.cv2_to_imgmsg(roi, "bgr8")
        self.debug_image_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
