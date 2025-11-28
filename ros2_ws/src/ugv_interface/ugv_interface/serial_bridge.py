import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import serial
import struct
import time
import math

# Constants
START_BYTE = 0xA5
END_BYTE = 0x5A

MSG_CMD_VEL = 0x01
MSG_ODOM = 0x02
MSG_IMU = 0x03
MSG_HEARTBEAT = 0x05

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to {port} at {baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect: {e}')
            exit(1)
            
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.create_timer(0.02, self.read_loop)
        self.create_timer(1.0, self.send_heartbeat)
        
        self.rx_buffer = bytearray()

    def calculate_crc8(self, data):
        crc = 0x00
        for byte in data:
            extract = byte
            for _ in range(8):
                sum_val = (crc ^ extract) & 0x01
                crc >>= 1
                if sum_val:
                    crc ^= 0x8C
                extract >>= 1
        return crc

    def send_packet(self, msg_id, payload=b''):
        packet = bytearray()
        packet.append(START_BYTE)
        packet.append(msg_id)
        packet.append(len(payload))
        packet.extend(payload)
        packet.append(self.calculate_crc8(payload))
        packet.append(END_BYTE)
        self.ser.write(packet)

    def cmd_vel_callback(self, msg):
        payload = struct.pack('<ff', msg.linear.x, msg.angular.z)
        self.send_packet(MSG_CMD_VEL, payload)

    def send_heartbeat(self):
        self.send_packet(MSG_HEARTBEAT)

    def read_loop(self):
        if self.ser.in_waiting > 0:
            self.rx_buffer.extend(self.ser.read(self.ser.in_waiting))
            
            while True:
                if len(self.rx_buffer) < 5: # Min size
                    break
                
                if self.rx_buffer[0] != START_BYTE:
                    self.rx_buffer.pop(0)
                    continue
                
                payload_len = self.rx_buffer[2]
                total_len = 3 + payload_len + 2
                
                if len(self.rx_buffer) < total_len:
                    break
                
                packet = self.rx_buffer[:total_len]
                
                if packet[-1] != END_BYTE:
                    self.rx_buffer.pop(0)
                    continue
                
                payload = packet[3:3+payload_len]
                crc = packet[-2]
                
                if self.calculate_crc8(payload) == crc:
                    self.process_packet(packet[1], payload)
                else:
                    self.get_logger().warn('CRC Mismatch')
                
                self.rx_buffer = self.rx_buffer[total_len:]

    def process_packet(self, msg_id, payload):
        if msg_id == MSG_ODOM:
            if len(payload) == 20:
                x, y, theta, v, w = struct.unpack('<fffff', payload)
                
                msg = Odometry()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'odom'
                msg.child_frame_id = 'base_link'
                msg.pose.pose.position.x = x
                msg.pose.pose.position.y = y
                # Theta to Quat
                msg.pose.pose.orientation.z = math.sin(theta / 2.0)
                msg.pose.pose.orientation.w = math.cos(theta / 2.0)
                
                msg.twist.twist.linear.x = v
                msg.twist.twist.angular.z = w
                
                self.odom_pub.publish(msg)
                
        elif msg_id == MSG_IMU:
            if len(payload) == 24:
                ax, ay, az, gx, gy, gz = struct.unpack('<ffffff', payload)
                
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = ay
                msg.linear_acceleration.z = az
                msg.angular_velocity.x = gx
                msg.angular_velocity.y = gy
                msg.angular_velocity.z = gz
                
                self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
