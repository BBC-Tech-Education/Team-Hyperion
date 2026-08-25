#ifndef TSSP_SYSTEM_H
#define TSSP_SYSTEM_H

#include <Arduino.h>
#include <common.h>
#include <pins.h>

class Tssp_system {
public:
    Tssp_system() {}; // Constructor
    void init();
    int read();
    int tsspStrength = 0;
    int tsspNum = 0;
private:
    static const int NUM_TSSPS = NUM_TSSP; // Number of TSSPs
    uint8_t Pin;
    uint8_t tsspCounter;
    int readTssp[NUM_TSSPS] = {0};
    int tsspPins[NUM_TSSPS] = {TSSP1, TSSP2, TSSP3, TSSP4, TSSP5, TSSP6, TSSP7, TSSP8, TSSP9, TSSP10, TSSP11, TSSP12}; //pins.h defining each pin
    int ignores[NUM_TSSPS] = {0};
    int largest = 0;
    int ballAngle = 0;
    int addedAngles = 0;
    int ballDirection;
    int check = 1;
    int prevBallDir = 0;
};

#endif