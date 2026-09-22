#ifndef BALL_HANDLING_H
#define BALL_HANDLING_H

#include <Arduino.h>
#include "Config.h"
#include "Pins.h"
#include "Timer.h"
#include "Voltage_divider.h"

class BallHandling {
public:
    BallHandling();
    void init();
    void update();
    void kick();
    bool photogate_triggered();
    bool can_kick();
    uint8_t get_current_kicks() { return kicks; };
    
private:
    void update_caps_led();

    VoltageDivider kickerVd;
    Timer pulseTimer;
    Timer rechargeTimer;
    Timer cooldownTimer;

    uint8_t kicks;
    bool isKicking;
    bool cooldownActive;
};

#endif