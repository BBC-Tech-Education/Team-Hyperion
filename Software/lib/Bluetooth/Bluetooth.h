#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "Config.h"
#include "Timer.h"

struct PeerData {
    bool role = false;
    bool enabled = false;
    uint8_t score = 0;
    uint8_t ballStr = 0;
    uint8_t batLvl = 0;
};

class Bluetooth {
public:
    void init();
    void update(uint8_t score, uint8_t ballStr, bool enabled, uint8_t batLvl);

    bool get_role() const { return self.role; }
    bool is_other_connected() const { return connected; }

private:
    void read();
    void send();
    void resolve_role();

    PeerData self;
    PeerData other;

    Timer connectedTimer = Timer(BT_CONNECTION_TIMEOUT_US);

    bool switching = false;
    bool connected = false;
    bool otherPrevRole = false;
};

#endif