#include "Bluetooth.h"

void Bluetooth::init() {
    BT_SERIAL.begin(BT_BAUD);
    connectedTimer.update();
}

void Bluetooth::update(bool enabled, Vect ball, Vect pos) {
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
            BT_SERIAL.read(); // Consume second start byte
            
            uint8_t info = BT_SERIAL.read();
            other.enabled = (info >> 1) & 0x01;
            other.role = info & 0x01;

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

bool Bluetooth::defender_can_steal(Vect defenderBall) {
    return defenderBall.exists()
        && defenderBall.isBetween(BALL_FRONT_MAX, BALL_FRONT_MIN)
        && defenderBall.mag < SWITCHING_STRENGTH;
}

void Bluetooth::calculate_role() {
    if (!self.enabled) {
        self.role = true; // Attacker
        return;
    }

    if (!connected || !other.enabled) {
        self.role = false; // Defender
        return;
    }

    if (CONTROL) {
        if (switchTimer.time_has_passed_no_update()) {
            Vect defenderBall = self.role ? other.ball : self.ball;
            if (defender_can_steal(defenderBall)) {
                self.role = !self.role;
                switchTimer.update();
            }
        }
    } else {
        self.role = !other.role;
    }

    #if DEBUG_BT_ROLE
        Serial.printf("self role:%d en:%d mag:%.1f | other role:%d en:%d mag:%.1f conn:%d\n",
                      self.role, self.enabled, self.ball.mag,
                      other.role, other.enabled, other.ball.mag, connected);
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
