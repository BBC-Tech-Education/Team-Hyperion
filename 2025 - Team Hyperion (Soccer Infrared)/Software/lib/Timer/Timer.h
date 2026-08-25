/*!
 * @file Timer.h
 * 
 * @mainpage This is a library for a inbuilt timer inside the code
 * 
 * @date 17/06/25
 * 
 * @author J.Huang (Brisbane Boys' College)
 */
#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

/*!
 * @brief Class that stores states and functions for interacting with the
 *        in-built Arduino timer.
 */
class Timer {
public:
    Timer(unsigned long duration);
    void update();
    bool timeHasPassed();
    bool timeHasPassedNoUpdate();

private:
    unsigned long timerDuration;
    unsigned long lastUpdate;
};

#endif