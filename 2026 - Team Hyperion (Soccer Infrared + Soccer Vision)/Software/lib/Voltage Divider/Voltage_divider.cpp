#include <Arduino.h>
#include "Voltage_divider.h"

void VoltageDivider::init() {
    pinMode(pin, INPUT);
}

float VoltageDivider::get_lvl() {
    return (analogRead(pin) / divider) + offset;
}
