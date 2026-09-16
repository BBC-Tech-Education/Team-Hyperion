#include "Bluetooth.h"

void Bluetooth::init() {
    BT_SERIAL.begin(BT_BAUD);
    connectedTimer.update();
}

void Bluetooth::update(bool enabled, Vect ball, Vect pos) {
    self.enabled = enabled;
    self.ball.i = (int16_t)ball.i;
    self.ball.j = (int16_t)ball.j;
    self.pos.i = (int16_t)pos.i;
    self.pos.j = (int16_t)pos.j;
    if(sendTimer.time_has_passed()) {
        send();
    }
    read();
    // Serial.println(other.pos.i);

    connected = !connectedTimer.time_has_passed_no_update();
    calculate_role();
}

void Bluetooth::read() {
    // Serial.println(BT_SERIAL.available());
    if(BT_SERIAL.available() >= BT_PACKET_SIZE) {
        uint8_t b1 = BT_SERIAL.read();
        uint8_t b2 = BT_SERIAL.peek();
        Serial.print(b1);
        Serial.print("\t");
        Serial.println(b2);
        if(b1 == BT_START_BYTE && b2 == BT_START_BYTE) {
            // Serial.println("hey sigma");
            BT_SERIAL.read();
            uint8_t info = BT_SERIAL.read();
            other.enabled = (info >> 1)&0x01;
            otherPreviousRole = other.role;
            other.role = info&0x01;

            other.ball.i = receive_vector_comp();
            other.ball.j = receive_vector_comp();
            other.pos.i = receive_vector_comp();
            other.pos.j = receive_vector_comp();

            switching = (otherPreviousRole != other.role) && (self.role == other.role);
            connectedTimer.update();
        }
    }
}

void Bluetooth::calculate_role() {
    // if (self.ballStr == 0) {
    //     switching = false;
    //     return;
    // }

    // if(!self.enabled) {
    //     self.role = true;
    //     roleConflict.update();
    // } else if(!connected || !other.enabled) {
    //     self.role = false;
    //     roleConflict.update();
    // } else if(switching) {
    //     self.role = !self.role;
    //     roleConflict.update();
    // } else if(self.role == other.role) {
    //     if(roleConflict.time_has_passed_no_update()) {
    //         self.role = self.ballStr < other.ballStr;
    //         roleConflict.update();
    //     }
    // } else if(!self.role && self.attackCone && (self.ballStr < SWITCHING_STRENGTH)) {
    //     switching = true;
    // }
}

void Bluetooth::send() {
    BT_SERIAL.write(BT_START_BYTE);
    BT_SERIAL.write(BT_START_BYTE);
    uint8_t info = ((self.enabled & 0x01) << 1) | (self.role & 0x01);
    BT_SERIAL.write(info);
    send_vector(self.ball);
    send_vector(self.pos);
}

int16_t Bluetooth::receive_vector_comp() {
    uint8_t highByte = BT_SERIAL.read();
    uint8_t lowByte = BT_SERIAL.read();
    uint16_t byte = (static_cast<uint16_t>(highByte) << 8) | lowByte;
    return static_cast<int16_t>(byte);
}

void Bluetooth::send_vector(Vect v) {
    int16_t iComp = static_cast<int16_t>(v.i);
    uint8_t highByte = (iComp >> 8) & 0xFF;
    uint8_t lowByte = iComp & 0xFF;
    BT_SERIAL.write(highByte);
    BT_SERIAL.write(lowByte);

    int16_t jComp = static_cast<int16_t>(v.j);
    highByte = (jComp >> 8) & 0xFF;
    lowByte = jComp & 0xFF;
    BT_SERIAL.write(highByte);
    BT_SERIAL.write(lowByte);
}