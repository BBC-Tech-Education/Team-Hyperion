#ifndef TSSP_SYSTEM_H
#define TSSP_SYSTEM_H

#include <Arduino.h>

class Tssp_system {
public:
    Tssp_system(int startPin); // Constructor
    void init();
    int read();
    int tsspStrength = 0;
private:
    static const int NUM_TSSPS = 12; // Number of TSSPs
    uint8_t Pin;
    int readTssp[NUM_TSSPS];
    int largest = 0;
    int tsspNum = 0;
    int ballAngle = 0;
    int addedAngles = 0;
};

#endif
