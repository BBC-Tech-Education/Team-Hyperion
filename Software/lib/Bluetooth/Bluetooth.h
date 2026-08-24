#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include "Timer.h"
#include "Config.h"

struct CommunicationData {
    bool role = false; // sent & read
    bool enabled = false; // sent & read
    bool attackCone = false; // sent & read
    uint8_t ballStr = 0; // sent & read
};

class Bluetooth {
public:
    void init();
    void update(bool enabled, float ballDir, float ballStr);

    bool get_role() { return self.role; };
private:
    void read();
    void send();
    void calculate_role();

    CommunicationData self;
    CommunicationData other;
    
    Timer sendTimer{BT_SEND_TIMER_US};
    Timer roleConflict{BT_ROLE_CONFLICT_TIMER_US};
    Timer connectedTimer{BT_CONNECTION_TIMEOUT_US};

    bool connected = false;
    bool otherPreviousRole = false;
    bool switching = false;
};

#endif