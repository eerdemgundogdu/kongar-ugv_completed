#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

// Protocol Constants
#define START_BYTE 0xA5
#define END_BYTE   0x5A

// Message IDs
enum MsgID : uint8_t {
    MSG_CMD_VEL = 0x01,
    MSG_ODOM    = 0x02,
    MSG_IMU     = 0x03,
    MSG_BATTERY = 0x04,
    MSG_HEARTBEAT = 0x05
};

// Structures
struct __attribute__((packed)) CmdVelMsg {
    float linear_x;
    float angular_z;
};

struct __attribute__((packed)) OdomMsg {
    float x;
    float y;
    float theta;
    float linear_vel;
    float angular_vel;
};

struct __attribute__((packed)) ImuMsg {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

class SerialProtocol {
public:
    SerialProtocol(Stream& stream);
    void update();
    void sendOdom(float x, float y, float theta, float lin_vel, float ang_vel);
    void sendImu(float ax, float ay, float az, float gx, float gy, float gz);
    void sendHeartbeat();
    
    bool isCmdVelAvailable();
    CmdVelMsg getCmdVel();

private:
    Stream& _stream;
    
    // Parsing state
    enum State {
        WAIT_START,
        WAIT_ID,
        WAIT_LEN,
        WAIT_PAYLOAD,
        WAIT_CRC,
        WAIT_END
    };
    
    State _state;
    uint8_t _msgId;
    uint8_t _payloadLen;
    uint8_t _payloadIndex;
    uint8_t _payloadBuffer[64];
    uint8_t _crc;
    
    CmdVelMsg _lastCmdVel;
    bool _cmdVelAvailable;
    
    void sendPacket(uint8_t msgId, uint8_t* payload, uint8_t len);
    uint8_t calculateCRC(uint8_t* data, uint8_t len);
    uint8_t updateCRC(uint8_t crc, uint8_t data);
};

#endif
