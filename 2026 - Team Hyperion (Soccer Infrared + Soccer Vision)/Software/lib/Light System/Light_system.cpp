#include <Arduino.h>
#include "Light_system.h"
#include "Common.h"


void LightSystem::read() {
    for (uint8_t i = 0; i < 16; i++) {
        for (uint8_t j = 0; j < 4; j++) {
            digitalWrite(inPin[j], (i >> j) & 0x01);
        }

        delayMicroseconds(10);

        sensorReadings[muxIndex[i]]      = analogRead(LS_OUT_1);
        sensorReadings[muxIndex[i + 16]] = analogRead(LS_OUT_2);
        sensorReadings[muxIndex[i + 32]] = analogRead(LS_OUT_3);
    }

    #if DEBUG_LS_RAW
    Serial.println("-----------VALUES----------");
    for (uint8_t i = 0; i < LS_INNER_NUM; i++) {
        Serial.print(sensorReadings[i] / 10);
        Serial.print("\t");
    }
    Serial.println();
    Serial.println();
    for (uint8_t i = LS_INNER_NUM; i < LS_NUM; i++) {
        Serial.print(sensorReadings[i] / 10);
        Serial.print("\t");
    }
    Serial.println();
    #endif
}


void LightSystem::init() {
    pinMode(LS_OUT_1, INPUT);
    pinMode(LS_OUT_2, INPUT);
    pinMode(LS_OUT_3, INPUT);

    for (uint8_t i = 0; i < 4; i++) {
        pinMode(inPin[i], OUTPUT);
    }

    read();

    for (uint8_t i = 0; i < LS_NUM; i++) {
        thresholds[i] = sensorReadings[i] + LS_THRESH;
    }
}



void LightSystem::update() {

    read();

    for (uint8_t i = 0; i < LS_NUM; i++) {
        onWhite[i] = (sensorReadings[i] > thresholds[i]);
    }

    #if DEBUG_LS_THRESH
    Serial.println("----------THRESHOLDS----------");
    for (uint8_t i = 0; i < LS_INNER_NUM; i++) {
        Serial.print(thresholds[i] / 10);
        Serial.print("\t");
    }
    Serial.println();
    Serial.println();
    for (uint8_t i = LS_INNER_NUM; i < LS_NUM; i++) {
        Serial.print(thresholds[i] / 10);
        Serial.print("\t");
    }
    Serial.println();
    #endif

    for (uint8_t i = 0; i < LS_INNER_NUM; i++) {
        if (!onWhite[i]) {
            onWhite[i] = (onWhite[mod(i - 1, LS_INNER_NUM)] && onWhite[mod(i + 1, LS_INNER_NUM)]);
        }
    }

    #if DEBUG_LS_ONWHITE
    Serial.println("----------ON WHITE----------");
    for (uint8_t i = 0; i < LS_INNER_NUM; i++) {
        Serial.print(onWhite[i]);
        Serial.print("\t");
    }
    Serial.println();
    for (uint8_t i = LS_INNER_NUM; i < LS_NUM; i++) {
        Serial.print(onWhite[i]);
        Serial.print("\t");
    }
    Serial.println();
    #endif

    clusterNum = 0;
    inCluster = false;
    LightSystem::Cluster clusterArray[4];
    lineDir = -1.0f;
    lineSize = -1.0f;

    for (uint8_t i = 0; i < LS_INNER_NUM; i++) {
        if (!inCluster) {
            if (onWhite[i]) {
                clusterArray[clusterNum].start = i;
                inCluster = true;
            }
        } else {
            if (!onWhite[i]) {
                clusterArray[clusterNum].end = i - 1;
                inCluster = false;
                clusterNum++;
            }
        }
    }

    if (onWhite[LS_INNER_NUM - 1]) {
        if (onWhite[0]) {
            clusterArray[0].start = clusterArray[clusterNum].start;
        } else {
            clusterArray[clusterNum].end = LS_INNER_NUM - 1;
            clusterNum++;
        }
    }

    if (clusterNum > 0) {
        for (uint8_t i = 0; i < 3; i++) {
            clusterArray[i].midpoint = mid_angle_between(clusterArray[i].start * 11.25f, clusterArray[i].end * 11.25f);
        }

        #if DEBUG_LS_CLUSTER
        Serial.printf("Number of Clusters: %d\t", clusterNum);
        for (uint8_t i = 0; i < clusterNum; i++) {
            Serial.printf("C%d: s=%d e=%d m=%.2f\t", i + 1, clusterArray[i].start, clusterArray[i].end, clusterArray[i].midpoint);
        }
        Serial.println();
        #endif

        if (clusterNum == 3) {
            float angleDiff12 = angle_between(clusterArray[0].midpoint, clusterArray[1].midpoint);
            float angleDiff23 = angle_between(clusterArray[1].midpoint, clusterArray[2].midpoint);
            float angleDiff31 = angle_between(clusterArray[2].midpoint, clusterArray[0].midpoint);
            float biggestAngle = fmaxf(angleDiff12, fmaxf(angleDiff23, angleDiff31));

            if(biggestAngle == angleDiff12) {
                lineDir = mid_angle_between(clusterArray[1].midpoint, clusterArray[0].midpoint);
                lineSize = angle_between(clusterArray[1].midpoint, clusterArray[0].midpoint) <= 180.0f ? 1.0f - cosf(DEG_TO_RAD * angle_between(clusterArray[1].midpoint, clusterArray[0].midpoint) / 2.0f) : 1.0f;
            } else if(biggestAngle == angleDiff23) {
                lineDir = mid_angle_between(clusterArray[2].midpoint, clusterArray[1].midpoint);
                lineSize = angle_between(clusterArray[2].midpoint, clusterArray[1].midpoint) <= 180.0f ? 1.0f - cosf(DEG_TO_RAD * angle_between(clusterArray[2].midpoint, clusterArray[1].midpoint) / 2.0f) : 1.0f;
            } else {
                lineDir = mid_angle_between(clusterArray[0].midpoint, clusterArray[2].midpoint);
                lineSize = angle_between(clusterArray[0].midpoint, clusterArray[2].midpoint) <= 180.0f ? 1.0f - cosf(DEG_TO_RAD * angle_between(clusterArray[0].midpoint, clusterArray[2].midpoint) / 2.0f) : 1.0f;
            }
        } else if (clusterNum == 2) {
            bool clockwise = angle_between(clusterArray[0].midpoint, clusterArray[1].midpoint) <= 180.0f;
            lineDir = clockwise ? mid_angle_between(clusterArray[0].midpoint, clusterArray[1].midpoint) : mid_angle_between(clusterArray[1].midpoint, clusterArray[0].midpoint);
            lineSize = 1.0f - cosf(DEG_TO_RAD * (clockwise ? angle_between(clusterArray[0].midpoint, clusterArray[1].midpoint) / 2.0f : angle_between(clusterArray[1].midpoint, clusterArray[0].midpoint) / 2.0f));
        } else {
            lineDir = clusterArray[0].midpoint;
            lineSize = 1.0f - cosf(DEG_TO_RAD * angle_between(clusterArray[0].start * 11.25f, clusterArray[0].end * 11.25f) / 2.0f);
        }
    } else {
        if (onWhite[32] || onWhite[33] || onWhite[34] || onWhite[35]) {
            clusterArray[0].midpoint = 0.0f;
            clusterNum ++;
        }

        if (onWhite[36] || onWhite[37] || onWhite[38] || onWhite[39]) {
            clusterArray[1].midpoint = 90.0f;
            clusterNum ++;
        }

        if (onWhite[40] || onWhite[41] || onWhite[42] || onWhite[43]) {
            clusterArray[2].midpoint = 180.0f;
            clusterNum ++;
        }

        if (onWhite[44] || onWhite[45] || onWhite[46] || onWhite[47]) {
            clusterArray[3].midpoint = 270.0f;
            clusterNum ++;
        }

        if (clusterNum == 2) {
            if (clusterArray[0].midpoint != -1.0f) {
                if (clusterArray[1].midpoint != -1.0f) {
                    lineDir = 45.0f;
                    lineSize = 0.1f;
                } else if (clusterArray[3].midpoint != -1.0f) {
                    lineDir = 315.0f;
                    lineSize = 0.1f;
                }
            } else if (clusterArray[2].midpoint != -1.0f) {
                if (clusterArray[1].midpoint != -1.0f) {
                    lineDir = 135.0f;
                    lineSize = 0.1f;
                } else if (clusterArray[3].midpoint != -1.0f) {
                    lineDir = 225.0f;
                    lineSize = 0.1f;
                }
            }
        } else if (clusterNum == 1) {
            for (uint8_t i = 0; i < 4; i++) {
                if (clusterArray[i].midpoint != -1.0f) {
                    lineDir = clusterArray[i].midpoint;
                }
            }
            lineSize = 0.1f;
        }
    }

    if (lineDir != -1.0f) {
        lineDir = float_mod(450.0f - lineDir, 360.0f);
    }

    #if DEBUG_LS_RAW
    Serial.printf("LS_RAW_VECT: dir=%.2f\tsize=%.2f\n", lineDir, lineSize);
    #endif

    #if DEBUG_LS
    Serial.println();
    #endif
}

float LightSystem::get_line_angle() {
    return lineDir;
}

float LightSystem::get_line_size() {
    return lineSize;
}
