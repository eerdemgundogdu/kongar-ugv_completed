import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.bridge = CvBridge()
        
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle/detected', 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO('yolov8n.pt')
                self.get_logger().info('YOLO loaded')
            except Exception as e:
                self.get_logger().warn(f'YOLO failed: {e}, using fallback')
                self.model = None
        else:
            self.get_logger().warn('No YOLO, using color')
            self.model = None
        
        self.get_logger().info('Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Bridge Error: {e}')
            return

        obstacle_detected = False
        
        if self.model is not None:
            results = self.model(cv_image, verbose=False)
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    if conf > 0.5:
                        obstacle_detected = True
                        break
                        
                if obstacle_detected:
                    break
        else:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            lower_red1 = (0, 100, 100)
            upper_red1 = (10, 255, 255)
            lower_red2 = (160, 100, 100)
            upper_red2 = (180, 255, 255)
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2
            
            if cv2.countNonZero(mask) > 500:
                obstacle_detected = True
            
        msg = Bool()
        msg.data = obstacle_detected
        self.obstacle_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
