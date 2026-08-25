#include "Tssp_system.h"
#include <pins.h>

void Tssp_system::init() {
    for (uint8_t i = 0; i < NUM_TSSPS; i++) {
        pinMode(tsspPins[i], INPUT);
    }
}

int Tssp_system::read() {
    largest = 0;
    tsspNum = 0;
    addedAngles = 0;
    tsspStrength = 0;
    ballAngle = 0;
    check = 1;
    for(int i = 0; i < NUM_TSSPS; i++) {
        readTssp[i] = 0;
    }
    for (int y = 0; y < 255; y++) {
        for (int i = 0; i < NUM_TSSPS; i++) {
            readTssp[i] += (1 - digitalRead(tsspPins[i]));
        }
        delayMicroseconds(3);
    }
    // for (int i = 0; i < NUM_TSSPS; i++) {
    //     Serial.print(readTssp[i]);
    //     // Serial.print(" Help: ");
    //     // Serial.print(ignores[i]);
    //     Serial.print(" ");
    // }
    Serial.println();
    if (readTssp[0] == 0 && readTssp[1] == 0 && readTssp[2] == 0 && readTssp[3] == 0 && readTssp[4] == 0 && readTssp[5] == 0 && readTssp[6] == 0 && readTssp[7] == 0 && readTssp[8] == 0 && readTssp[9] == 0 && readTssp[10] == 0 && readTssp[11] == 0 && readTssp[12] == 0) {
        check = 0;
    }
    for(uint8_t i = 0; i < NUM_TSSPS; i++){
        ignores[i] = (readTssp[i] == 255 || readTssp[i] == -1) ? 1 : 0;
    }
    for (int i = 0; i < NUM_TSSPS; i++) {
        if(ignores[i] == 0) {
            if (readTssp[i] > largest) {
                largest = readTssp[i];
                tsspNum = i;
            } 
        }
    }
    for (int i = 0; i < NUM_TSSPS; i++) {
        if(ignores[i] == 0){
            addedAngles += readTssp[i];
        }
    }
    // Serial.print("tsspIgnores: ");
    // for (int i = 0; i < NUM_TSSPS; i++) {
    //     Serial.print(ignores[i]);
    //     Serial.print(" ");
    // }
    // Serial.println("");
    if(tsspNum != 1 || tsspNum != 0) {
        tsspNum++;
    }
    tsspStrength = addedAngles/NUM_TSSPS;
    ballDirection = (tsspNum-1)*30;
    prevBallDir = ballDirection;
    return ballDirection;
}