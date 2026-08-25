#include "light_system.h"

void Light_System::init() {
    for (int i = 0; i < NUM_MUX; i++) {
        pinMode(muxList[i], OUTPUT);
    }
    pinMode(LIGHT_PIN, INPUT);
}

void Light_System::read() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        mux(i);
        lightData[i] = analogRead(Pin);
    }
}

void Light_System::mux(int muxPin) {
    for (int i = 0; i < NUM_MUX; i++) {
        digitalWrite(muxList[i], muxPin%2);
        muxPin = int(muxPin / 2);
    }
}
