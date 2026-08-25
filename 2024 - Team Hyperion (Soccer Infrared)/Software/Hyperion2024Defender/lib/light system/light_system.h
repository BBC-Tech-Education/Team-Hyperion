#ifndef LIGHT_SYSTEM_H
#define LIGHT_SYSTEM_H

#include <Arduino.h>
#include <pins.h>

class Light_System {
public:
    Light_System() {}; // Constructor
    void init();
    void read();
    void mux(int muxPin);
private:
    static const int NUM_SENSORS = 16; // Number of Light Sensors
    static const int NUM_MUX = 4;
    uint8_t Pin;
    int pin_mux;
    int lightData[NUM_SENSORS];
    int muxList[4] = {LIGHT_PIN_DIGI_1, LIGHT_PIN_DIGI_2, LIGHT_PIN_DIGI_3, LIGHT_PIN_DIGI_4};
};

#endif
