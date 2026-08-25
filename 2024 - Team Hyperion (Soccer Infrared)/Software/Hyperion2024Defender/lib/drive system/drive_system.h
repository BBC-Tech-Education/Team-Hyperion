#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <Motor.h>

class Drive_system {
public:
    Drive_system() {};// Constructor
    void init();
    void run_all(int speed, int dir, int rotation);
private:
    static const int NUM_MOTORS = 4; // Number of motors
    static const int motor_pins[NUM_MOTORS][3]; // Pin configurations for each motor
    Motor motors[NUM_MOTORS] = {Motor(motor_pins[0][0], motor_pins[0][1], motor_pins[0][2]), 
                                Motor(motor_pins[1][0], motor_pins[1][1], motor_pins[1][2]),
                                Motor(motor_pins[2][0], motor_pins[2][1], motor_pins[2][2]),
                                Motor(motor_pins[3][0], motor_pins[3][1], motor_pins[3][2])
                                };
    float hiJudges[4];
    int motorFace1 = 135*DEG_TO_RAD;
    int motorFace2 = 225*DEG_TO_RAD;
    int motorFace3 = 315*DEG_TO_RAD;
    int motorFace4 = 45*DEG_TO_RAD;
    int a[4] = {motorFace1, motorFace2, motorFace3, motorFace4};
    int largest = 0;
};

#endif
