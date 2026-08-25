/*!
 * @file LSLB.h
 *
 * This is a library to calculate the lines direction for outs
 *
 * T.McCabe (Brisbane Boys' College)
 * S.Garg (Brisbane Boys' College)
 */
#ifndef LSystem_H
#define LSystem_H
 
#include <Arduino.h>
#include <config.h>
#include <common.h>
#include <pins.h>
#include <math.h>
 
/*!
 * @brief Class that stores state and functions for calculating the direction of
          the line for outs
 */
class Light_system {
public:
    Light_system() {};
    void init();
    void update(float rot, bool motorOn);

    float getLineState();
    float getLineDirection();
private:
    int readOne(int sensor_num);
    float lineState = 0;
    float lineDir = -1;
    void calculateLineState(float rot, float lineDirection, float linePos);
    float calculateDistanceOver(float angle1, float angle2);
    int whiteThreshold[NUM_LS] = {0};
    int pinList[4] = {LIGHT_PIN_DIGI_0, LIGHT_PIN_DIGI_1, LIGHT_PIN_DIGI_2,
                     LIGHT_PIN_DIGI_3};
    bool sensorIsWhite[NUM_LS] = {0};
    int maxIndex = 44;
    int minIndex = 44;
    int clustersList[4][2];
    float clusterCenter[3] = {44, 44, 44};
};
 
#endif
