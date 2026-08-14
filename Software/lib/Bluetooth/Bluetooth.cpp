#include <Arduino.h>
#include "Bluetooth.h"

void Bluetooth::init() {
    BT_SERIAL.begin(BT_BAUD);
    connectedTimer.update();
}

void Bluetooth::update(uint8_t score, uint8_t ballStr, bool enabled, uint8_t batLvl) {
    self.score = score;
    self.ballStr = ballStr;
    self.enabled = enabled;
    self.batLvl = batLvl;

    read();
    connected = !connectedTimer.time_has_passed_no_update();
    resolve_role();
    send();
}

void Bluetooth::resolve_role() {
    if (self.ballStr == 0 && other.ballStr == 0) {
        switching = false;
        return;
    }
    if (!self.enabled) {
        self.role = true;
        return;
    }
    if (!connected || !other.enabled) {
        self.role = false;
        return;
    }
    if (switching) {
        self.role = !self.role;
        switching = false;
        return;
    }
    if (self.batLvl < ROBOT_REQUIRED_VOLT && other.batLvl < ROBOT_REQUIRED_VOLT && self.batLvl != other.batLvl) {
        self.role = (self.batLvl > other.batLvl);
        return;
    }

    self.role = (self.score >= other.score);

#if DEBUG_BT_ROLE
    Serial.printf("BT_ROLE: conn=%d role=%d self=%d other=%d\n",
                  connected, self.role, self.score, other.score);
#endif
}

void Bluetooth::read() {
    while (BT_SERIAL.available() >= BT_PACKET_SIZE) {
        if (BT_SERIAL.read() != BT_START_BYTE) {
            continue;
        }
        if (BT_SERIAL.peek() != BT_START_BYTE) {
            continue;
        }
        BT_SERIAL.read();

        otherPrevRole = other.role;

        uint8_t flags = BT_SERIAL.read();
        other.role = flags & BT_FLAG_ROLE;
        other.enabled = (flags >> 1) & 0x01;
        switching = (otherPrevRole != other.role) && (self.role == other.role);

        other.score = BT_SERIAL.read();
        other.ballStr = BT_SERIAL.read();
        other.batLvl = BT_SERIAL.read();

        connectedTimer.update();

#if DEBUG_BT_RX
        Serial.printf("BT_RX: role=%d en=%d score=%d ballStr=%d bat=%d\n",
                      other.role, other.enabled, other.score, other.ballStr, other.batLvl);
#endif
    }
}

void Bluetooth::send() {
    uint8_t packet[BT_PACKET_SIZE] = {
        BT_START_BYTE,
        BT_START_BYTE,
        (uint8_t)((self.enabled << 1) | (self.role & 0x01)),
        self.score,
        self.ballStr,
        self.batLvl,
    };
    BT_SERIAL.write(packet, BT_PACKET_SIZE);

#if DEBUG_BT_TX
    Serial.printf("BT_TX: role=%d en=%d score=%d ballStr=%d bat=%d\n",
                  self.role, self.enabled, self.score, self.ballStr, self.batLvl);
#endif
}
