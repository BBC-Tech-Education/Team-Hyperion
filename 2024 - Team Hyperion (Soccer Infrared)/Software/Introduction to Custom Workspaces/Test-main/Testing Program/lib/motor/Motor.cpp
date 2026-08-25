#include "Motor.h"

Motor::Motor(int pwm_pin, int dir_a_pin, int dir_b_pin) {
    pwm_pin = pwm;
    dir_a_pin = dir_a;
    dir_b_pin = dir_b;
}

void Motor::init() {
    pinMode(pwm, OUTPUT);
    pinMode(dir_a, OUTPUT);
    pinMode(dir_b, OUTPUT);
}

int Motor::run(int speed, int direction_a, int direction_b) {
    analogWrite(pwm, speed);
    digitalWrite(dir_a, direction_a);
    digitalWrite(dir_b, direction_b);
}