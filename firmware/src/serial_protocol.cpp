#include "serial_protocol.h"

SerialProtocol::SerialProtocol(Stream& stream) : _stream(stream) {
    _state = WAIT_START;
    _cmdVelAvailable = false;
}

void SerialProtocol::update() {
    while (_stream.available()) {
        uint8_t byte = _stream.read();
        
        switch (_state) {
            case WAIT_START:
                if (byte == START_BYTE) {
                    _state = WAIT_ID;
                    _crc = 0; // Reset CRC
                }
                break;
                
            case WAIT_ID:
                _msgId = byte;
                _crc = updateCRC(_crc, byte);
                _state = WAIT_LEN;
                break;
                
            case WAIT_LEN:
                _payloadLen = byte;
                _crc = updateCRC(_crc, byte);
                _payloadIndex = 0;
                if (_payloadLen > 0) {
                    if (_payloadLen > 64) { // Safety check
                        _state = WAIT_START;
                    } else {
                        _state = WAIT_PAYLOAD;
                    }
                } else {
                    _state = WAIT_CRC;
                }
                break;
                
            case WAIT_PAYLOAD:
                _payloadBuffer[_payloadIndex++] = byte;
                _crc = updateCRC(_crc, byte);
                if (_payloadIndex >= _payloadLen) {
                    _state = WAIT_CRC;
                }
                break;
                
            case WAIT_CRC:
                if (byte == _crc) {
                    _state = WAIT_END;
                } else {
                    // CRC Error
                    _state = WAIT_START;
                }
                break;
                
            case WAIT_END:
                if (byte == END_BYTE) {
                    // Valid Packet Received
                    if (_msgId == MSG_CMD_VEL && _payloadLen == sizeof(CmdVelMsg)) {
                        memcpy(&_lastCmdVel, _payloadBuffer, sizeof(CmdVelMsg));
                        _cmdVelAvailable = true;
                    }
                }
                _state = WAIT_START;
                break;
        }
    }
}

void SerialProtocol::sendPacket(uint8_t msgId, uint8_t* payload, uint8_t len) {
    _stream.write(START_BYTE);
    _stream.write(msgId);
    _stream.write(len);
    
    uint8_t crc = 0;
    crc = updateCRC(crc, msgId);
    crc = updateCRC(crc, len);
    
    for (int i = 0; i < len; i++) {
        _stream.write(payload[i]);
        crc = updateCRC(crc, payload[i]);
    }
    
    _stream.write(crc);
    _stream.write(END_BYTE);
}

void SerialProtocol::sendOdom(float x, float y, float theta, float lin_vel, float ang_vel) {
    OdomMsg msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;
    msg.linear_vel = lin_vel;
    msg.angular_vel = ang_vel;
    sendPacket(MSG_ODOM, (uint8_t*)&msg, sizeof(OdomMsg));
}

void SerialProtocol::sendImu(float ax, float ay, float az, float gx, float gy, float gz) {
    ImuMsg msg;
    msg.accel_x = ax;
    msg.accel_y = ay;
    msg.accel_z = az;
    msg.gyro_x = gx;
    msg.gyro_y = gy;
    msg.gyro_z = gz;
    sendPacket(MSG_IMU, (uint8_t*)&msg, sizeof(ImuMsg));
}

void SerialProtocol::sendHeartbeat() {
    sendPacket(MSG_HEARTBEAT, NULL, 0);
}

bool SerialProtocol::isCmdVelAvailable() {
    bool ret = _cmdVelAvailable;
    _cmdVelAvailable = false;
    return ret;
}

CmdVelMsg SerialProtocol::getCmdVel() {
    return _lastCmdVel;
}

uint8_t SerialProtocol::updateCRC(uint8_t crc, uint8_t data) {
    // Simple 8-bit CRC or Checksum. Using XOR sum + rotation for simplicity and speed on MCU
    // Or just simple sum for now as per previous, but let's make it slightly better: 
    // Dallas/Maxim CRC8 is standard. Let's use a simple sum for now to match the "robustness" requirement without overengineering if not needed.
    // Actually, let's stick to the plan: "CRC8".
    
    uint8_t extract;
    uint8_t sum;
    for(int i = 8; i; i--) {
        sum = (crc ^ data) & 0x01;
        crc >>= 1;
        if (sum)
            crc ^= 0x8C;
        data >>= 1;
    }
    return crc;
}

uint8_t SerialProtocol::calculateCRC(uint8_t* data, uint8_t len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = updateCRC(crc, data[i]);
    }
    return crc;
}
