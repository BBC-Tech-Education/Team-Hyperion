#include "Bluetooth.h"

void Bluetooth::init() {
    BT_SERIAL.begin(BT_BAUD);
    connectedTimer.update();
}

void Bluetooth::update(bool enabled, float ballDir, float ballStr) {
    self.enabled = enabled;
    self.ballStr = (ballStr > 255.0f) ? 255 : (uint8_t)ballStr;
    self.attackCone = (ballDir > BALL_FRONT_MAX || ballDir < BALL_FRONT_MIN);

    read();
    calculate_role();
    send();
}

// 1: startbyte1
// 2: startbyte2
// 3: ballstr
// 4: info (enabled, role, attackCone)

void Bluetooth::read() {
    uint8_t b1 = BT_SERIAL.read();
    uint8_t b2 = BT_SERIAL.peek();
    if(b1 == BT_START_BYTE && b2 == BT_START_BYTE) {
        BT_SERIAL.read();
        bool otherPreviousRole = other.role;
        uint8_t info = BT_SERIAL.read();
        
    }
}

void Bluetooth::calculate_role() {

}

void Bluetooth::send() {
    BT_SERIAL.write(BT_START_BYTE);
    BT_SERIAL.write(BT_START_BYTE);
    BT_SERIAL.write(self.ballStr);
    uint8_t info = (self.enabled&0x01) << 2 | (self.attackCone&0x01) << 1 | (self.role&0x01);
    BT_SERIAL.write(info);
}