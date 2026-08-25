#ifndef TSSP_H
#define TSSP_H

#include <Arduino.h>

class Tssp {
public:
    Tssp(int p);
    void init();
    int read();
private:
    int pin;
};
#endif