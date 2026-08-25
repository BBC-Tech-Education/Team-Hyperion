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

void Motor::run(int speed) {
    if (speed > 0 ) {
        digitalWrite(dir_a, HIGH);
        digitalWrite(dir_b, LOW);
    } else if (speed < 0) {
        digitalWrite(dir_a, LOW);
        digitalWrite(dir_b, HIGH);
    } else {
        digitalWrite(dir_a, LOW);
        digitalWrite(dir_b, LOW);
    }
    analogWrite(pwm, abs(speed));
}