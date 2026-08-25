/*!
 * @file bluetooth.cpp
 */
#include <Bluetooth.h>

/*!
 * @brief Initializes bluetooth module for usage.
 */
void Bluetooth::init() {
    BT_SERIAL.begin(BT_BAUD);
    connectedTimer.update();
    roleConflict.update();
    sendTimer.update();
}

/*!
 * @brief Main controlling function of the bluetooth modules.
 * 
 * @param ballDir Current Direction of Ball.
 * @param ballStr Current Distance of Ball away from robot (units).
 */
void Bluetooth::update(float ballDir, float ballStr, bool enabled) {
    self.ballDir = ballDir;
    self.ballStr = ballStr;
    self.enabled = enabled;
    if(sendTimer.timeHasPassed()) {
        send();
    }
    read();

    bool connected = !connectedTimer.timeHasPassedNoUpdate();
    
    // 🛑 stop switching if ball is not detected by either robot
    if (self.ballStr == 0 && other.ballStr == 0) {
        switching = false;
        return;
    }

    if(!self.enabled) {
        self.role = 1;
        roleConflict.update();
    } else if(!connected || !other.enabled) {
        self.role = 0;
        roleConflict.update();
    } else if(switching) {
        self.role = !self.role;
        roleConflict.update();
    } else if(self.role == other.role) {
        if(roleConflict.timeHasPassedNoUpdate()) {
            self.role = self.ballStr > other.ballStr;
            roleConflict.update();
        }
    } else if(!self.role && ((self.ballDir < 15 || self.ballDir > 345) && (self.ballStr > SWITCHING_STRENGTH))) {
        switching = true;
    }


    #if DEBUG_BLUETOOTH
        Serial.print("SELF - role: ");
        Serial.print(self.role);
        Serial.print(" en: ");
        Serial.print(self.enabled);
        Serial.print(" ballDir: ");
        Serial.print(self.ballDir);
        Serial.print(" ballStr: ");
        Serial.print(self.ballStr);
        Serial.print(" ||| OTHER - role: ");
        Serial.print(other.role);
        Serial.print(" en:");
        Serial.print(other.enabled);
        Serial.print(" balDir: ");
        Serial.print(other.ballDir);
        Serial.print(" ballStr: ");
        Serial.println(other.ballStr);
    #endif
}

/*!
 * @brief If the serial has more than one full packet, it attempts to read the
          bluetooth module.
 */
void Bluetooth::read() {
   while(BT_SERIAL.available() >= BT_PACKET_SIZE) {
        uint8_t byte1 = BT_SERIAL.read();
        uint8_t byte2 = BT_SERIAL.peek();
        if(byte1 == BT_START_BYTE && byte2 == BT_START_BYTE) {
            BT_SERIAL.read();
            bool otherPrevRole = other.role;
            uint8_t info = BT_SERIAL.read();
            other.enabled = info >> 4;
            other.role = info % 2;
            switching = (otherPrevRole != other.role) && (self.role == other.role);
            byte1 = BT_SERIAL.read();
            byte2 = BT_SERIAL.read();
            uint16_t tempBallDir = (byte1 << 8) | byte2;
            other.ballDir = tempBallDir / 100.0f;
            other.ballStr = BT_SERIAL.read();
            connectedTimer.update();
        }
    }
}

/*!
 * @brief Writes data to the bluetooth module for other bluetooth devices to
 *        read off (other bluetooth modules).
 */
void Bluetooth::send() {
    BT_SERIAL.write(BT_START_BYTE);
    BT_SERIAL.write(BT_START_BYTE);
    uint8_t info = (self.enabled&0x01) << 4 | (self.role&0x01);
    BT_SERIAL.write(info);
    uint16_t b = self.ballDir * 100;
    BT_SERIAL.write(highByte(b));
    BT_SERIAL.write(lowByte(b));
    BT_SERIAL.write(self.ballStr);
}

/*! 
 * @brief Get function for role of robot (Attack/Defend).
 * 
 * @returns Role of the robot.
 */
bool Bluetooth::getRole() {
    return self.role;
}