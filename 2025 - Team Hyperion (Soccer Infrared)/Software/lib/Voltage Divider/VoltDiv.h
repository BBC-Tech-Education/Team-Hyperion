/*!
 * @file VoltDiv.h
 * 
 * @mainpage This is a library used to read the battery level of the robot.
 * 
 * @author S.Garg (Brisbane Boys' College)
 */
#ifndef VOLTDIV_H
#define VOLTDIV_H

#include <Arduino.h>

/*!
 * @brief Class that stores state and functions for interacting with the
 *        robot's electrical circuit using voltage dividers to uncover battery
 *        level.
 */
class VoltDiv { 
public:
    VoltDiv(uint8_t p, float d) : pin(p), divider(d) {}
    void init();
    void update();
    float getLvl();
    bool getMotorOn();
private:
    uint8_t pin;
    float divider;
    float lvl;
    bool motorOn;
};

#endif // VOLTDIV_H