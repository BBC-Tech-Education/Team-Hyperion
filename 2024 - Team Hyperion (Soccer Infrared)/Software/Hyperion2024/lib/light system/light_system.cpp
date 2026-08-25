#include "light_system.h"

void Light_System::init() {
    for (int i = 0; i < NUM_MUX; i++) {
        pinMode(muxList[i], OUTPUT);
    }
    pinMode(LIGHT_PIN, INPUT);

    read();

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        uint16_t calibrateTotal = 0;
        for (uint8_t j = 0; j < 16; j++)
        {
            calibrateTotal += lightData[i];
            //delay(1); !! REMMEBER TO UN COMMENT OUT
            // Serial.print(read[i]);
            // Serial.print("  ");
        }
        ls_cal[i] = (uint16_t)round(calibrateTotal/16) + thresh;
        // Serial.println(ls_cal[i]);
    } 
}

void Light_System::read() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        digitalWrite(muxList[0], (i >> 0) & 0x01);
        digitalWrite(muxList[1], (i >> 1) & 0x01);
        digitalWrite(muxList[2], (i >> 2) & 0x01);
        digitalWrite(muxList[3], (i >> 3) & 0x01);
        lightData[i] = analogRead(LIGHT_PIN);
        white[i] = (lightData[i] >= ls_cal[i]);
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (white[(i+1)%NUM_SENSORS] && white[(i-1)%NUM_SENSORS]) {
            white[i] = true;
        }
    }
    // for (int data: white) {
    //     Serial.print(data);
    //     Serial.print("\t");
    // }
    // Serial.println();
    // Serial.println(lightData[17]);
}

float Light_System::update() { //change to do logic AFTER reading (not in the function)
    read();
    uint8_t clusterNum = 0;
    bool inCluster = false;
    Light_System::Cluster clusterArray[4];
    for (int j = 0; j < NUM_SENSORS; j++) { // reads sensors
        if (!inCluster) { // checks for white 
            if (white[j]) { // checks if it is in cluster
                inCluster = true;
                clusterNum++;
                clusterArray[clusterNum].start = j;
            } 
        } else { // if no longer in white
            if (!white[j]) { // checks if in cluster
                inCluster = false;
                clusterArray[clusterNum].end = j - 1;
                // clusterArray[clusterNum].midpoint = -1 * (midAngleBetween(clusterArray[clusterNum].start * (360 / NUM_SENSORS), clusterArray[clusterNum].end * (360 / NUM_SENSORS))) + 360;
            }
        }
    }
    if(white[15]){
        if(white[0]){
            clusterArray[0].start = clusterArray[clusterNum - 1].start;
        } else {
            clusterArray[clusterNum - 1].end = 15;
        }
    }

    switch (clusterNum)
    {
    case 0:
        lineAngle = -1;
        break;
    case 1:
        lineAngle = 360 - floatMod(clusterArray[0].midpoint - LS_OFFSET, 360);
        break;
    case 2:
        if (angleBetween(clusterArray[1].midpoint, clusterArray[0].midpoint) <= 180)
        {
            lineAngle = 360 - floatMod(midAngleBetween(clusterArray[1].midpoint, clusterArray[0].midpoint) - LS_OFFSET, 360);
        }
        else
        {
            lineAngle = 360 - floatMod(midAngleBetween(clusterArray[0].midpoint, clusterArray[1].midpoint) - LS_OFFSET, 360);
        }
        break;
    case 3:
        float angleDiff12 = angleBetween(clusterArray[1].midpoint, clusterArray[0].midpoint);
        float angleDiff23 = angleBetween(clusterArray[2].midpoint, clusterArray[1].midpoint);
        float angleDiff31 = angleBetween(clusterArray[0].midpoint, clusterArray[2].midpoint);
        float biggestAngle = fmax(angleDiff12, fmax(angleDiff23, angleDiff31));
        if (biggestAngle == angleDiff12)
        {
            lineAngle = 360 - floatMod(midAngleBetween(clusterArray[0].midpoint, clusterArray[1].midpoint) + LS_OFFSET, 360);
        }
        else if (biggestAngle == angleDiff23)
        {
            lineAngle = 360 - floatMod(midAngleBetween(clusterArray[1].midpoint, clusterArray[2].midpoint) + LS_OFFSET, 360);
        }
        else
        {
            lineAngle = 360 - floatMod(midAngleBetween(clusterArray[2].midpoint, clusterArray[0].midpoint) + LS_OFFSET, 360);
        }
        break;
    }

    Serial.println(lineAngle);

    // bool countStatement = true; //true = clockwise, false = anticlockwise
    // for (int i = 0; i < NUM_SENSORS; i++) {
    //     //positiveData[i] = (lightData[i] >= white[i]);
    // }
    // for (int i = 0; i < NUM_SENSORS; i++) {
    //     if (positiveData[i] != 0) {
    //         isLineAvoidance = false;
    //         break;
    //     } else {
    //         isLineAvoidance = true;
    //     }
    // }
    // if (positiveData[15]) {
    //     countStatement = false;
    //     for (int i = NUM_SENSORS; i > 0; i--) {
    //         if (positiveData[i-1] != 1) {
    //             firstSens = i+1;
    //         }
    //     }
    // } else {
    //     for (int i = 0; i < NUM_SENSORS; i++) {
    //         if (positiveData[i] == 1) {
    //             firstSens = i+1;
    //             break;
    //         }
    //     }
    // }
    // if (countStatement == false) {
    //     for (int i = 0; i < NUM_SENSORS; i++) {
    //         if (positiveData[(firstSens+i) >= 16? firstSens+i-16 : firstSens+i] != 1) {
    //             lastSens = firstSens+i-16;
    //         }
    //     }
    // } else {
    //     for (int i = 0; i < NUM_SENSORS; i++) {
    //         if (positiveData[firstSens+i] != 1) {
    //             lastSens = firstSens+i;
    //         }
    //     }
    // }
    // midSens = (firstSens+lastSens)/2;
    // if (firstSens < lastSens) {
    //     fdegs = ((midSens-1)*22.5)-180;
    // } else {
    //     fdegs = ((midSens-1)*22.5);
    // }
    // if (fdegs == 0) {
    //     fdegs = 1;
    // }

    return lineAngle;
}