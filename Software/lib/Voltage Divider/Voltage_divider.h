/*!
 * @file VoltDiv.h
 * 
 * @mainpage This is a library used to read the battery level of the robot.
 * 
 * @author S.Garg (Brisbane Boys' College)
 */
#ifndef VOLTDIV_H
#define VOLTDIV_H

/*!
 * @brief Class that stores state and functions for interacting with the
 *        robot's electrical circuit using voltage dividers to uncover battery
 *        level.
 */
class VoltageDivider { 
public:
    VoltageDivider(uint8_t p, float d, float o) : pin(p), divider(d), offset(o) {}
    void init();
    float get_lvl();
private:
    uint8_t pin;
    float divider;
    float offset;
};

#endif