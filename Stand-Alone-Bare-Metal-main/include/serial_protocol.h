#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdint.h>

// Protocol Constants
#define START_BYTE 0xA5
#define END_BYTE   0x5A

// Message IDs
#define MSG_CMD_VEL   0x01
#define MSG_ODOM      0x02
#define MSG_IMU       0x03
#define MSG_BATTERY   0x04
#define MSG_HEARTBEAT 0x05

// Data Structures (Packed)
#pragma pack(push, 1)
struct CmdVelMsg {
    float linear_x;
    float angular_z;
};

struct OdomMsg {
    float x;
    float y;
    float theta;
    float linear_vel;
    float angular_vel;
};

struct ImuMsg {
    float ax, ay, az;
    float gx, gy, gz;
};
#pragma pack(pop)

class SerialProtocol {
public:
    SerialProtocol();
    void update();
    
    bool isCmdVelAvailable();
    CmdVelMsg getCmdVel();
    
    void sendOdom(float x, float y, float theta, float v, float w);
    void sendImu(float ax, float ay, float az, float gx, float gy, float gz);
    void sendHeartbeat();

private:
    uint8_t calculateCRC8(const uint8_t *data, size_t len);
    void sendPacket(uint8_t msgId, const uint8_t *payload, size_t len);
    
    // State Machine
    enum State { WAIT_START, READ_ID, READ_LEN, READ_PAYLOAD, READ_CRC, READ_END };
    State state;
    
    uint8_t msgId;
    uint8_t payloadLen;
    uint8_t payloadBuffer[64];
    uint8_t payloadIndex;
    
    CmdVelMsg lastCmdVel;
    bool cmdVelAvailable;
};

#endif
