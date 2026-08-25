#include "Light_Sensor.h"
Light_Sensor::Light_Sensor(int p) {
    pin = p;
}

void Light_Sensor::init() {
    pinMode(pin, INPUT);
}

int Light_Sensor::read() {
    int value = analogRead(pin);
    return value;
}