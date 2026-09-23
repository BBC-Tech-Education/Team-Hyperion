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
    void update(float ballStr = 0.0f);
    void kick();
    bool photogate_triggered();
    bool can_kick();
    uint8_t get_current_kicks() { return kicks; };

    uint16_t photogateThresh = 0;
   
private:
    void update_caps_led();
    void run_dribbler(float spd);
 
    VoltageDivider kickerVd;
    Timer pulseTimer;
    Timer rechargeTimer;
    Timer cooldownTimer;
 
    uint8_t kicks;
    bool isKicking;
    bool cooldownActive;
};
 
#endif