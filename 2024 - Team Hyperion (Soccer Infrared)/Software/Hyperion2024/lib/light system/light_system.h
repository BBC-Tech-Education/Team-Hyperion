#ifndef LIGHT_SYSTEM_H
#define LIGHT_SYSTEM_H

#include <Arduino.h>
#include <pins.h>
#include <common.h>

class Light_System {
public:
    Light_System() {}; // Constructor
    void init();
    void read();
    void setMuxChannel(int channel);
    float update();
    bool isLineAvoidance = false;
    float lineAngle = -1;
    int lightData[NUM_LS] = {0};
private:
    static const int NUM_SENSORS = NUM_LS; // Number of Light Sensors
    static const int NUM_MUX = NUMBER_MUX;
    uint8_t pin;
    int pin_mux;
    int muxList[4] = {LIGHT_PIN_DIGI_0, LIGHT_PIN_DIGI_1, LIGHT_PIN_DIGI_2, LIGHT_PIN_DIGI_3};
    bool positiveData[NUM_SENSORS] = {0};
    double ls_cal[NUM_SENSORS] = {0};
    int white[NUM_SENSORS] = {0};
    int thresh = 100;
    struct Cluster {
        float start = -1;
        float end = -1;
        float midpoint = -1;
    };
    // int firstSens = 0;
    // int lastSens = 0;
    // int difference = 0;
    // int movementAngle = 0;
    // int sensDif = 0;
    // int rsensDif = 0;
    // int midSens = 0;
    // int fdegs = 0;
};

#endif
