#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include "Timer.h"
#include "Config.h"
#include "Vect.h"

struct CommunicationData {
    bool role = false; // sent & read
    bool enabled = false; // sent & read
    Vect ball{0.0f, 0.0f, false};
    Vect pos{0.0f, 0.0f, false};
};

class Bluetooth {
public:
    void init();
    void update(bool enabled, Vect ball, Vect pos);

    bool get_role() { return self.role; };
    Vect get_other_pos() { return other.pos; };
    Vect get_other_ball() { return other.ball; };
    bool get_other_role() { return other.role; };
private:
    void read();
    void send();
    void send_vector(Vect v);
    void calculate_role();
    bool defender_can_steal(Vect defenderBall);
    int16_t receive_vector_comp();

    CommunicationData self;
    CommunicationData other;
    
    Timer sendTimer{BT_SEND_TIMER_US};
    Timer switchTimer{BT_SWITCH_TIMER_US};
    Timer connectedTimer{BT_CONNECTION_TIMEOUT_US};

    bool connected = false;
};

#endif
