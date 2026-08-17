#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <Timer.h>

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
    
    Timer pairedTimer(BT_CONNECTION_TIMEOUT_US);
};

#endif