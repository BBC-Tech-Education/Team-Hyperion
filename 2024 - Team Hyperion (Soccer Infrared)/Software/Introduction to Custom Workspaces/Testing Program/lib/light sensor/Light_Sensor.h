#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include <Arduino.h>

class Light_Sensor {
public:
    Light_Sensor(int p);
    void init();
    int read();
private:
    int pin;
};
#endif