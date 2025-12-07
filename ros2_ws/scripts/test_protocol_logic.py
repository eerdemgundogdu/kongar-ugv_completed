#!/usr/bin/env python3
"""
Unit tests for the serial protocol CRC8 implementation.
Tests both Python (ROS2 side) and validates protocol format.
"""

import struct
import unittest

# CRC8 Implementation (same as in serial_bridge.py and firmware)
def calculate_crc8(data):
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

# Protocol Constants
START_BYTE = 0xA5
END_BYTE = 0x5A

MSG_CMD_VEL = 0x01
MSG_ODOM = 0x02
MSG_IMU = 0x03
MSG_BATTERY = 0x04
MSG_HEARTBEAT = 0x05


def create_packet(msg_id, payload=b''):
    """Create a valid protocol packet."""
    packet = bytearray()
    packet.append(START_BYTE)
    packet.append(msg_id)
    packet.append(len(payload))
    packet.extend(payload)
    
    # CRC is calculated over msg_id, len, and payload (matches firmware and serial_bridge.py)
    crc_data = bytearray([msg_id, len(payload)]) + payload
    crc = calculate_crc8(crc_data)
    packet.append(crc)
    packet.append(END_BYTE)
    return bytes(packet)


def parse_packet(data):
    """Parse a protocol packet and return (msg_id, payload, valid)."""
    if len(data) < 5:
        return None, None, False
    
    if data[0] != START_BYTE or data[-1] != END_BYTE:
        return None, None, False
    
    msg_id = data[1]
    payload_len = data[2]
    
    if len(data) != 3 + payload_len + 2:
        return None, None, False
    
    payload = data[3:3+payload_len]
    crc = data[-2]
    
    crc_data = bytearray([msg_id, payload_len]) + payload
    expected_crc = calculate_crc8(crc_data)
    
    if crc != expected_crc:
        return msg_id, payload, False
    
    return msg_id, payload, True


class TestCRC8(unittest.TestCase):
    def test_empty_data(self):
        result = calculate_crc8(b'')
        self.assertEqual(result, 0x00)
    
    def test_single_byte(self):
        result = calculate_crc8(b'\x00')
        self.assertIsInstance(result, int)
        self.assertTrue(0 <= result <= 255)
    
    def test_deterministic(self):
        data = b'Hello, UGV!'
        result1 = calculate_crc8(data)
        result2 = calculate_crc8(data)
        self.assertEqual(result1, result2)
    
    def test_different_data(self):
        result1 = calculate_crc8(b'ABC')
        result2 = calculate_crc8(b'XYZ')
        self.assertNotEqual(result1, result2)


class TestPacketCreation(unittest.TestCase):
    def test_heartbeat_packet(self):
        packet = create_packet(MSG_HEARTBEAT)
        self.assertEqual(packet[0], START_BYTE)
        self.assertEqual(packet[1], MSG_HEARTBEAT)
        self.assertEqual(packet[2], 0)  # No payload
        self.assertEqual(packet[-1], END_BYTE)
        self.assertEqual(len(packet), 5)
    
    def test_cmd_vel_packet(self):
        linear = 0.5
        angular = 0.3
        payload = struct.pack('<ff', linear, angular)
        packet = create_packet(MSG_CMD_VEL, payload)
        
        self.assertEqual(packet[0], START_BYTE)
        self.assertEqual(packet[1], MSG_CMD_VEL)
        self.assertEqual(packet[2], 8)  # 2 floats = 8 bytes
        self.assertEqual(packet[-1], END_BYTE)
    
    def test_odom_packet(self):
        x, y, theta, v, w = 1.0, 2.0, 0.5, 0.2, 0.1
        payload = struct.pack('<fffff', x, y, theta, v, w)
        packet = create_packet(MSG_ODOM, payload)
        
        self.assertEqual(packet[2], 20)  # 5 floats = 20 bytes


class TestPacketParsing(unittest.TestCase):
    def test_parse_heartbeat(self):
        packet = create_packet(MSG_HEARTBEAT)
        msg_id, payload, valid = parse_packet(packet)
        
        self.assertTrue(valid)
        self.assertEqual(msg_id, MSG_HEARTBEAT)
        self.assertEqual(payload, b'')
    
    def test_parse_cmd_vel(self):
        linear, angular = 0.25, -0.5
        payload = struct.pack('<ff', linear, angular)
        packet = create_packet(MSG_CMD_VEL, payload)
        
        msg_id, parsed_payload, valid = parse_packet(packet)
        
        self.assertTrue(valid)
        self.assertEqual(msg_id, MSG_CMD_VEL)
        
        parsed_linear, parsed_angular = struct.unpack('<ff', parsed_payload)
        self.assertAlmostEqual(parsed_linear, linear, places=5)
        self.assertAlmostEqual(parsed_angular, angular, places=5)
    
    def test_parse_odom(self):
        x, y, theta, v, w = 1.5, -0.5, 1.57, 0.3, 0.0
        payload = struct.pack('<fffff', x, y, theta, v, w)
        packet = create_packet(MSG_ODOM, payload)
        
        msg_id, parsed_payload, valid = parse_packet(packet)
        
        self.assertTrue(valid)
        self.assertEqual(msg_id, MSG_ODOM)
        
        px, py, pt, pv, pw = struct.unpack('<fffff', parsed_payload)
        self.assertAlmostEqual(px, x, places=5)
        self.assertAlmostEqual(py, y, places=5)
        self.assertAlmostEqual(pt, theta, places=5)
    
    def test_corrupted_crc(self):
        packet = bytearray(create_packet(MSG_HEARTBEAT))
        original_crc = packet[-2]
        packet[-2] = (original_crc + 1) % 256  # Ensure different CRC
        
        msg_id, payload, valid = parse_packet(bytes(packet))
        self.assertFalse(valid)
    
    def test_truncated_packet(self):
        packet = create_packet(MSG_HEARTBEAT)
        truncated = packet[:-1]  # Remove END_BYTE
        
        msg_id, payload, valid = parse_packet(truncated)
        self.assertFalse(valid)


class TestIMUPacket(unittest.TestCase):
    def test_imu_packet(self):
        ax, ay, az = 0.0, 0.0, 9.81
        gx, gy, gz = 0.1, -0.1, 0.0
        payload = struct.pack('<ffffff', ax, ay, az, gx, gy, gz)
        packet = create_packet(MSG_IMU, payload)
        
        msg_id, parsed_payload, valid = parse_packet(packet)
        
        self.assertTrue(valid)
        self.assertEqual(msg_id, MSG_IMU)
        self.assertEqual(len(parsed_payload), 24)
        
        pax, pay, paz, pgx, pgy, pgz = struct.unpack('<ffffff', parsed_payload)
        self.assertAlmostEqual(paz, 9.81, places=2)


class TestBatteryPacket(unittest.TestCase):
    def test_battery_packet(self):
        battery_level = 87.5
        payload = struct.pack('<f', battery_level)
        packet = create_packet(MSG_BATTERY, payload)
        
        msg_id, parsed_payload, valid = parse_packet(packet)
        
        self.assertTrue(valid)
        self.assertEqual(msg_id, MSG_BATTERY)
        
        parsed_level = struct.unpack('<f', parsed_payload)[0]
        self.assertAlmostEqual(parsed_level, battery_level, places=1)


if __name__ == '__main__':
    print("=" * 60)
    print("UGV Serial Protocol Unit Tests")
    print("=" * 60)
    
    unittest.main(verbosity=2)
