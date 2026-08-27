#ifndef LIGHT_SYSTEM_H
#define LIGHT_SYSTEM_H

#include "Pins.h"
#include "Config.h"
#include "Vect.h"


class LightSystem {
public:
    void init();
    void update();

    Vect get_line_vector() {return lineVector;};
    float get_line_angle();
    float get_line_size();

private:
    void read();

    struct Cluster {
        int8_t start = -1;
        float midpoint = -1.0f;
        int8_t end = -1;
    };

    uint8_t inPin[4] = {LS_IN_0, LS_IN_1, LS_IN_2, LS_IN_3};

    uint8_t muxIndex[LS_NUM] = {
        40, 14, 41, 13, 42, 12, 43, 11, 15, 10, 16, 39, 17, 38, 18,  9, // Mux 1
         3,  4,  5,  6, 36, 37,  7,  8, 35, 34, 33, 32,  2,  1,  0, 31, // Mux 2
        24, 30, 20, 26, 22, 28, 44, 47, 23, 29, 19, 25, 21, 27, 45, 46  // Mux 3
    };

    uint16_t thresholds[LS_NUM];
    uint16_t sensorReadings[LS_NUM];
    uint8_t onWhite[LS_NUM];

    uint8_t clusterNum;
    bool inCluster;

    float lineDir;
    float lineSize;
    Vect lineVector;
};

#endif