import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class VisualOdometry(Node):
    def __init__(self):
        super().__init__('visual_odometry')
        
        self.bridge = CvBridge()
        
        self.odom_pub = self.create_publisher(Odometry, '/visual_odom', 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        self.prev_gray = None
        self.prev_kp = None
        
        # Camera Matrix
        self.K = np.array([[500, 0, 320],
                           [0, 500, 240],
                           [0, 0, 1]], dtype=np.float32)
                           
        self.cur_R = np.eye(3)
        self.cur_t = np.zeros((3, 1))
        
        self.get_logger().info('Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Bridge Error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        if self.prev_gray is None:
            self.prev_gray = gray
            # Init features
            orb = cv2.ORB_create()
            kp = orb.detect(gray, None)
            self.prev_kp = cv2.KeyPoint_convert(kp)
            return

        if self.prev_kp is None or len(self.prev_kp) < 10:
             orb = cv2.ORB_create()
             kp = orb.detect(gray, None)
             self.prev_kp = cv2.KeyPoint_convert(kp)
             self.prev_gray = gray
             return

        # Optical Flow
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_kp, None)
        
        # Select points
        if p1 is not None:
            good_new = p1[st==1]
            good_old = self.prev_kp[st==1]
            
            if len(good_new) > 10:
                # Est Essential Mat
                E, mask = cv2.findEssentialMat(good_new, good_old, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
                
                if E is not None:
                    # Recover Pose
                    _, R, t, mask = cv2.recoverPose(E, good_new, good_old, self.K)
                    
                    # Update Pose
                    scale = 1.0 
                    
                    self.cur_t = self.cur_t + scale * self.cur_R.dot(t)
                    self.cur_R = R.dot(self.cur_R)
                    
                    self.publish_odometry()

            self.prev_gray = gray
            self.prev_kp = good_new.reshape(-1, 1, 2)

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Position
        odom.pose.pose.position.x = float(self.cur_t[0])
        odom.pose.pose.position.y = float(self.cur_t[1])
        odom.pose.pose.position.z = float(self.cur_t[2])
        
        # Orientation
        yaw = math.atan2(self.cur_R[1, 0], self.cur_R[0, 0])
        
        # Quat from Yaw
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
