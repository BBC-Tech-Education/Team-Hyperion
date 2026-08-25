#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <Motor.h>
#include <pins.h>
#include <common.h>

class Drive_system {
    public:
        Drive_system() {}
        void init();
        void update(float speed, float angle, float heading, float correction); // angle relative to FIELD NOT ROBOT
    private:
        uint8_t inA[MOTOR_NUM] = {FRINA, BRINA, BLINA, FLINA};
        uint8_t inB[MOTOR_NUM] = {FRINB, BRINB, BLINB, FLINB};
        uint8_t pwm[MOTOR_NUM] = {FRPWM, BRPWM, BLPWM, FLPWM};
        float values[MOTOR_NUM] = {0};
        int angles[MOTOR_NUM] = {FRANGLE, BRANGLE, BLANGLE, FLANGLE};

};

#endif