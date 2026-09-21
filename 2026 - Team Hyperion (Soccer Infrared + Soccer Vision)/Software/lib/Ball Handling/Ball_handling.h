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
    bool can_kick();
    int get_current_kicks();

private:
    VoltageDivider kickerVd;
    Timer pulseTimer;
    Timer rechargeTimer;
    Timer cooldownTimer;

    int kicks;
    bool isKicking;
    bool cooldownActive;

    bool ball_held();
};

#endif