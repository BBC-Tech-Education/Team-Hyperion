#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(int pwm_pin, int dir_a_pin, int dir_b_pin);
    void init();
    int run(int speed, int direction_a, int direction_b);
private:
    int pwm;
    int dir_a;
    int dir_b;
};
#endif