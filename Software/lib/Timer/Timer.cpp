/**
 * @file Timer.cpp
 * @brief Soft timer implementation using Arduino micros().
 *
 * @note micros() overflows ~70 minutes; subtraction still works for intervals
 * shorter than that window due to unsigned wrap semantics.
 */

#include <Arduino.h>
#include "Timer.h"

Timer::Timer(unsigned long duration) {
    timerDuration = duration;
}

void Timer::update() {
    lastUpdate = micros();
}

bool Timer::time_has_passed() {
    if (micros() - lastUpdate > timerDuration) {
        update();
        return true;
    }

    return false;
}

bool Timer::time_has_passed_no_update() {
    return micros() - lastUpdate > timerDuration;
}
