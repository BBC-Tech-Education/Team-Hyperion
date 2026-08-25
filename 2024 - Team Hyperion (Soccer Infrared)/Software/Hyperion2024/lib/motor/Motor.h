#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    Motor(int pwm_pin, int dir_a_pin, int dir_b_pin);
    void init();
    void run(int speed);
private:    
    int pwm;
    int dir_a;
    int dir_b;
};

//int MotorSpeeds[1][3] = {{100, HIGH, LOW}}
#endif