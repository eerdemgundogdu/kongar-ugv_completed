import struct

# Protocol Constants
START_BYTE = 0xA5
END_BYTE = 0x5A

MSG_CMD_VEL = 0x01
MSG_ODOM = 0x02
MSG_IMU = 0x03
MSG_BATTERY = 0x04
MSG_HEARTBEAT = 0x05

def update_crc(crc, data):
    for _ in range(8):
        sum_val = (crc ^ data) & 0x01
        crc >>= 1
        if sum_val:
            crc ^= 0x8C
        data >>= 1
    return crc

def create_packet(msg_id, payload):
    length = len(payload)
    packet = bytes([START_BYTE, msg_id, length])
    
    crc = 0
    crc = update_crc(crc, msg_id)
    crc = update_crc(crc, length)
    
    for b in payload:
        crc = update_crc(crc, b)
        
    packet += payload
    packet += bytes([crc, END_BYTE])
    return packet

def parse_packet(packet):
    if len(packet) < 5:
        return False, "Too short"
    
    if packet[0] != START_BYTE:
        return False, "Invalid Start Byte"
    
    msg_id = packet[1]
    length = packet[2]
    
    if len(packet) != 3 + length + 2:
        return False, "Length mismatch"
        
    payload = packet[3:3+length]
    received_crc = packet[-2]
    end_byte = packet[-1]
    
    if end_byte != END_BYTE:
        return False, "Invalid End Byte"
        
    calc_crc = 0
    calc_crc = update_crc(calc_crc, msg_id)
    calc_crc = update_crc(calc_crc, length)
    for b in payload:
        calc_crc = update_crc(calc_crc, b)
        
    if calc_crc != received_crc:
        return False, f"CRC Mismatch: {calc_crc} != {received_crc}"
        
    return True, (msg_id, payload)

def test_protocol():
    print("Testing Protocol Logic...")
    
    # Test 1: Heartbeat
    print("Test 1: Heartbeat")
    packet = create_packet(MSG_HEARTBEAT, b'')
    valid, result = parse_packet(packet)
    if valid and result[0] == MSG_HEARTBEAT:
        print("PASS")
    else:
        print(f"FAIL: {result}")
        
    # Test 2: Odom
    print("Test 2: Odom")
    # x=1.0, y=2.0, theta=3.14, v=0.5, w=0.1
    payload = struct.pack('<fffff', 1.0, 2.0, 3.14, 0.5, 0.1)
    packet = create_packet(MSG_ODOM, payload)
    valid, result = parse_packet(packet)
    if valid and result[0] == MSG_ODOM:
        unpacked = struct.unpack('<fffff', result[1])
        print(f"PASS: {unpacked}")
    else:
        print(f"FAIL: {result}")

    # Test 3: CRC Error
    print("Test 3: CRC Error Injection")
    packet = bytearray(create_packet(MSG_HEARTBEAT, b''))
    packet[-2] = (packet[-2] + 1) % 256 # Corrupt CRC
    valid, result = parse_packet(packet)
    if not valid:
        print(f"PASS: Detected error - {result}")
    else:
        print("FAIL: Did not detect CRC error")

if __name__ == "__main__":
    test_protocol()
