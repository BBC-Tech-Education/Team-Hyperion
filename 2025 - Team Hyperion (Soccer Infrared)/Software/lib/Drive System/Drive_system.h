/*!
 * @file drive_system.h
 * 
 * This is a library that calculates the speed of the 4 motors based on a
 * singular direction that the robot should move in.
 * 
 * S.Garg (Brisbane Boys' College)
 */
#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include <config.h>
#include <common.h>
#include <pins.h>
#include <math.h>

/*!
 * @brief Class that stores state and functions for calculating the direction
         relative to speed for 4 motors.
 */
class Drive_system {
public:
    Drive_system() {};
    void init();
    void run(float speed, float angle, float correction);
    
private: 
    uint8_t motorInA[MOTORNUM] = {FRINA, FLINA, BLINA, BRINA};
    uint8_t motorInB[MOTORNUM] = {FRINB, FLINB, BLINB, BRINB};
    uint8_t motorPWM[MOTORNUM] = {FRPWM, FLPWM, BLPWM, BRPWM};
    float motorAngles[MOTORNUM] = {45.0, 135.0, 225.0, 315.0};
    float values[MOTORNUM] = {0.0};
};

#endif