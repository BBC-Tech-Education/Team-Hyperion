#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include "Config.h"
#include "Pins.h"

class DriveSystem {
public:
    DriveSystem() {};
    void init();
    void run(float spd, float ang, float cor);

private:
    uint8_t motorInA[MOTOR_NUM] = {FRINA, FLINA, BLINA, BRINA};
    uint8_t motorInB[MOTOR_NUM] = {FRINB, FLINB, BLINB, BRINB};
    uint8_t motorPWM[MOTOR_NUM] = {FRPWM, FLPWM, BLPWM, BRPWM};
    float motorAng[MOTOR_NUM] = {45.0, 135.0, 225.0, 315.0};
    float values[MOTOR_NUM] = {0.0};
};

#endif