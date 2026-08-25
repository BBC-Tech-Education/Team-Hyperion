#ifndef ORBIT_H
#define ORBIT_H

#include <Arduino.h>

class Orbit {
public:
    Orbit() {};
    int calculate_Direction(int ballAngle);
    int calculate_Speed(int ballStrength, float tsspBallAngle);
    int calculate_Direction2(int ballAngle2);
    int calculate_Direction3(int ballAngle3);
    int ballAngleOffset = 0;
private:
    int speed = 0;
    int orbitDirections[8] = {0, 90, 180, 270, 90, 180, 270, 0};
    float ORBIT_MULTIPLIER = 2.71828182846;
    float thing = 0;
    float powpow = 0;
    int returnVal = 0;
};

#endif