#include "Bluetooth.h"

void Bluetooth::init() {
    BT_SERIAL.begin(BT_BAUD);
    connectedTimer.update();
    roleConflict.update();
}

void Bluetooth::update(bool enabled, Vect ball, Vect pos, bool kickerReady) {
    self.enabled = enabled;

    int16_t iComponent = ball.i;
    int16_t jComponent = ball.j;
    self.ball.setStandard(iComponent, jComponent);

    iComponent = pos.i;
    jComponent = pos.j;
    self.pos.setStandard(iComponent, jComponent);

    if (sendTimer.time_has_passed()) {
        send();
    }
    read();

    connected = !connectedTimer.time_has_passed_no_update();
    calculate_role();
}

void Bluetooth::read() {
    while (BT_SERIAL.available() >= BT_PACKET_SIZE) {
        uint8_t b1 = BT_SERIAL.read();
        uint8_t b2 = BT_SERIAL.peek();
        if (b1 == BT_START_BYTE && b2 == BT_START_BYTE) {
            BT_SERIAL.read();
            
            bool otherPrevRole = other.role;
            
            uint8_t info = BT_SERIAL.read();
            other.enabled = (info >> 1) & 0x01;
            other.role = info & 0x01;

            switching = (otherPrevRole != other.role) && (self.role == other.role);

            int16_t iComp = receive_vector_comp();
            int16_t jComp = receive_vector_comp();
            other.ball.setStandard(iComp, jComp);

            iComp = receive_vector_comp();
            jComp = receive_vector_comp();
            other.pos.setStandard(iComp, jComp);

            connectedTimer.update();
        }
    }
}

void Bluetooth::calculate_role() {    
    if ((self.ball.mag == 0.0f) && (other.ball.mag == 0.0f)) {
        switching = false;
        return;
    }

    if (!self.enabled) {
        self.role = true;
        roleConflict.update();
    } else if (!connected || !other.enabled) {
        self.role = false;
        roleConflict.update();
    } else if (switching) {
        self.role = !self.role;
        roleConflict.update();
    } else if (self.role == other.role) {
        if (roleConflict.time_has_passed_no_update()) {
            self.role = self.ball.mag < other.ball.mag; 
            roleConflict.update();
        }
    } else if (!self.role && (self.ball.isBetween(345.0f, 15.0f) && (self.ball.mag > SWITCHING_STRENGTH))) {
        switching = true;
    }

    // ask raj when we turn on bt, for some reason the bt does not seem to connect to the other robot even after quite a
    // while e.g. when a robot is on the field (defending) and we put the other robot on the field, they do not pair/connect
    //            to each other AT ALL (like we have never seen it before and we waited like 2 mins)

    // when I did pair them: I made sure: BAUD Rate is the same, ADDR was assigned as the same, One was slave (0) one was master (1)

    #if DEBUG_BT_ROLE
        Serial.printf("self role:%d en:%d mag:%.1f | other role:%d en:%d mag:%.1f conn:%d sw:%d\n",
                      self.role, self.enabled, self.ball.mag,
                      other.role, other.enabled, other.ball.mag, connected, switching);
    #endif
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
    uint16_t combined = ((uint16_t)highByte << 8) | lowByte;
    return combined;
}

void Bluetooth::send_vector(Vect v) {
    int16_t iComp = v.i;
    uint8_t highByte = (iComp >> 8) & 0xFF;
    uint8_t lowByte = iComp & 0xFF;
    BT_SERIAL.write(highByte);
    BT_SERIAL.write(lowByte);

    int16_t jComp = v.j;
    highByte = (jComp >> 8) & 0xFF;
    lowByte = jComp & 0xFF;
    BT_SERIAL.write(highByte);
    BT_SERIAL.write(lowByte);
}