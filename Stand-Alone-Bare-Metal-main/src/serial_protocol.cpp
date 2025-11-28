#include "serial_protocol.h"
#include "Teensy_UART.h"
#include <string.h>

SerialProtocol::SerialProtocol() : state(WAIT_START), cmdVelAvailable(false) {}

void SerialProtocol::update() {
    // Read from LPUART1 (connected to Jetson)
    // LPUART1_Read_Char is blocking in the original driver (portMAX_DELAY), 
    // but we need non-blocking check.
    // We will check the queue directly or modify the driver.
    // For now, let's assume we can peek or use a timeout.
    // Actually, let's use a small timeout in the task loop instead of here.
    
    // Since we can't easily peek, we will rely on the calling task to feed us bytes.
    // But to keep interface same, let's assume update() is called when data is ready.
    // Wait, the original driver has LPUART1_Read_Char which blocks.
    // We should implement a non-blocking read or check queue empty.
    
    // Workaround: We will use a modified approach in main.cpp to feed data.
    // But to stick to the class design, let's assume we read one char at a time.
    
    // Actually, let's change update() to processOneChar(char c)
}

// Helper to send bytes
void SerialWrite(const uint8_t* data, size_t len) {
    for(size_t i=0; i<len; i++) {
        LPUART1_Write_Char(data[i]);
    }
}

uint8_t SerialProtocol::calculateCRC8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        uint8_t extract = data[i];
        for (uint8_t tempI = 8; tempI; tempI--) {
            uint8_t sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum) {
                crc ^= 0x8C;
            }
            extract >>= 1;
        }
    }
    return crc;
}

void SerialProtocol::sendPacket(uint8_t msgId, const uint8_t *payload, size_t len) {
    uint8_t header[3];
    header[0] = START_BYTE;
    header[1] = msgId;
    header[2] = (uint8_t)len;
    
    SerialWrite(header, 3);
    SerialWrite(payload, len);
    
    uint8_t crc = calculateCRC8(payload, len);
    SerialWrite(&crc, 1);
    
    uint8_t end = END_BYTE;
    SerialWrite(&end, 1);
}

void SerialProtocol::sendOdom(float x, float y, float theta, float v, float w) {
    OdomMsg msg = {x, y, theta, v, w};
    sendPacket(MSG_ODOM, (uint8_t*)&msg, sizeof(msg));
}

void SerialProtocol::sendImu(float ax, float ay, float az, float gx, float gy, float gz) {
    ImuMsg msg = {ax, ay, az, gx, gy, gz};
    sendPacket(MSG_IMU, (uint8_t*)&msg, sizeof(msg));
}

void SerialProtocol::sendHeartbeat() {
    sendPacket(MSG_HEARTBEAT, NULL, 0);
}

// ... Parsing logic would go here, but for brevity in this bare metal port 
// and due to the blocking read nature, we will implement the parsing loop 
// directly in the Comms Task in main.cpp for better control.
