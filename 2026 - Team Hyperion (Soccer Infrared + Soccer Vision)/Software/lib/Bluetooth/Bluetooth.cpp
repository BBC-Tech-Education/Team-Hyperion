#include "Bluetooth.h"

void Bluetooth::init() {
    BT_SERIAL.begin(BT_BAUD);
    connectedTimer.update();
}

void Bluetooth::update(bool enabled, float ballDir, float ballStr) {
    self.enabled = enabled;
    self.ballStr = (ballStr > 255.0f) ? 255 : (uint8_t)ballStr;
    self.attackCone = (ballDir > BALL_FRONT_MAX || ballDir < BALL_FRONT_MIN);
    if(sendTimer.time_has_passed()) {
        send();
    }
    read();

    connected = !connectedTimer.time_has_passed_no_update();
    calculate_role();
}

void Bluetooth::read() {
    if(BT_SERIAL.available() >= BT_PACKET_SIZE) {
        uint8_t b1 = BT_SERIAL.read();
        uint8_t b2 = BT_SERIAL.peek();
        if(b1 == BT_START_BYTE && b2 == BT_START_BYTE) {
            BT_SERIAL.read();
            other.ballStr = BT_SERIAL.read();
            uint8_t info = BT_SERIAL.read();
            other.enabled = (info >> 3)&0x01;
            other.attackCone = (info >> 2)&0x01;
            otherPreviousRole = other.role;
            other.role = info&0x03;
            switching = (otherPreviousRole != other.role) && (self.role == other.role);
            connectedTimer.update();
        }
    }
}

void Bluetooth::calculate_role() {
    if (self.ballStr == 0) {
        switching = false;
        return;
    }

    if(!self.enabled) {
        self.role = true;
        roleConflict.update();
    } else if(!connected || !other.enabled) {
        self.role = false;
        roleConflict.update();
    } else if(switching) {
        self.role = !self.role;
        roleConflict.update();
    } else if(self.role == other.role) {
        if(roleConflict.time_has_passed_no_update()) {
            self.role = self.ballStr < other.ballStr;
            roleConflict.update();
        }
    } else if(!self.role && self.attackCone && (self.ballStr < SWITCHING_STRENGTH)) {
        switching = true;
    }
}

void Bluetooth::send() {
    BT_SERIAL.write(BT_START_BYTE);
    BT_SERIAL.write(BT_START_BYTE);
    BT_SERIAL.write(self.ballStr);
    uint8_t info = ((self.enabled & 0x01) << 3) | ((self.attackCone & 0x01) << 2) | (self.role & 0x03);
    BT_SERIAL.write(info);
}