#ifndef INITIALIZE_H
#define INITIALIZE_H

#include <Arduino.h>

class initialize {
public:
    initialize() {};
    void calculate_final_movement();
    void reading_values();
    void initialize_sensors();
    void printing();
    void logic();
private:
    int a = 0;
};

#endif