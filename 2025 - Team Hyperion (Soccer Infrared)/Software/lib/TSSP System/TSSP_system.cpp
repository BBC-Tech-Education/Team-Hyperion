/*!
 * @file tssp_system.cpp
 * 
 * @mainpage An array of TSSP58038 Sensors
 * 
 * This is a library for an array of TSSP50838 Sensors
 */
#include "Tssp_system.h"

/*!
 * @brief Initialize all the tssp's accordingly.
 */
void Tssp_system::init() {
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        pinMode(tsspPins[i], INPUT);
    }
    // Assign tssp x and y components based on place in unit circle (vs real)
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        tsspX[i] = cosf(DEG_TO_RAD * (450 - i * (360 / TSSPNUM)));
        tsspY[i] = sinf(DEG_TO_RAD * (450 - i * (360 / TSSPNUM)));
    }
}

/*! 
 * @brief Completes calculations for ballDir and ballStr by reading the Tssps.
 */
void Tssp_system::update() {
    uint8_t readTssp[TSSPNUM] = {0};
    uint8_t tsspSortedValues[TSSPNUM] = {0};
    uint8_t tsspSortedIndex[TSSPNUM] = {0}; 

    // Read the tssps themselves.
    for(uint8_t i = 0; i < 255; i++) {
        for(uint8_t j = 0; j < TSSPNUM; j++) {
            readTssp[j] += 1 - digitalRead(tsspPins[j]);
        }
        delayMicroseconds(3);
    }


    for(uint8_t i = 0; i < TSSPNUM; i++) {
        if(readTssp[i] > 0) {
            readTssp[i] += offset[i];
        }
    }

    // Prints each individual sensor value out if true.
    #if DEBUG_TSSP_VALS 
        for(uint8_t i = 0; i < TSSPNUM; i++) {
            Serial.print(readTssp[i]);
            Serial.print(" ");
        }
        Serial.println();
    #endif

    // tsspSortedValues: Sorts all readTssp values in descending order
    // tsspSortedIndex: Sorts index's of readTssp in descending
    //                  order corresponding to the value it reads.
    for(uint8_t i = 0; i < TSSPNUM; i++) {
        for(uint8_t j = 0; j < TSSPNUM; j++) {
            if(readTssp[i] > tsspSortedValues[j]) {
                if(j <= i) {
                    ARRAYSHIFTDOWN(tsspSortedValues, j, i);
                    ARRAYSHIFTDOWN(tsspSortedIndex, j, i);
                }
                tsspSortedValues[j] = readTssp[i];
                tsspSortedIndex[j] = i;
                break;
            }
        }
    }

    // Average the top 4 readings from each of the sensors by their
    // corresponding x and y components
    float x = ((tsspSortedValues[0] * tsspX[tsspSortedIndex[0]]) + \
              (tsspSortedValues[1] * tsspX[tsspSortedIndex[1]]) + \
              (tsspSortedValues[2] * tsspX[tsspSortedIndex[2]]) + \
              (tsspSortedValues[3] * tsspX[tsspSortedIndex[3]])) / 4;
    float y = ((tsspSortedValues[0] * tsspY[tsspSortedIndex[0]]) + \
              (tsspSortedValues[1] * tsspY[tsspSortedIndex[1]]) + \
              (tsspSortedValues[2] * tsspY[tsspSortedIndex[2]]) + \
              (tsspSortedValues[3] * tsspY[tsspSortedIndex[3]])) / 4;
    
    // ballStr: Weighted average of the top 4 sensors.
    // detectingBall: If ball strength is 0 (all sensors read 0), then ball isnt 
    //                there.
    // ballDir: Using vector calculations, a certain radian value can be found 
    //          and then that is converted to degrees.
    ballStr = ((3 * tsspSortedValues[0]) + (2 * tsspSortedValues[1]) + 
                        tsspSortedValues[2] + tsspSortedValues[3]) / 7.0;
    ballDir = (ballStr != 0) ? 360 - \
                            floatMod((RAD_TO_DEG * (atan2f(y, x)))-90, 360) : 0;
}

void Tssp_system::orbit() {
    if(ballStr != 0) {
        float modBallDir = ballDir > 180 ? ballDir - 360 : ballDir;
        float moveScaler = constrain(ballStr /
                        ORBIT_STRENGTH_RADIUS, 0, 1);
        moveScaler = constrain((0.02 * moveScaler * expf(4.5 * moveScaler)), 0, 1);
        float moveOffset = moveScaler * min(0.4 * expf(0.25 * abs(modBallDir))
                        - 0.4, 90.0);
        moveDir = floatMod((modBallDir < 0 ? -moveOffset : moveOffset) + ballDir, 360.0);
        moveSpeed = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0 - moveOffset / 90.0);
        if((ballDir < 30.0 || ballDir > 330.0) && ballStr > 110.0) {
            moveDir = ballDir;
            moveSpeed = SURGE_SPEED+40;
        }
    } else {
        moveDir = 0.0;
        moveSpeed = 0.0;
    }
}

/*!
 * @brief Get function for the how far away the ball is relative to the robot
 *        in units.
 * 
 * @returns Ball Strength (how far away the ball is) of the array of TSSPs.
 */
float Tssp_system::getBallStr() {
    return ballStr;
}

/*!
 * @brief Get function for the direction of the ball relative to the robot.
 * 
 * @returns Direction of the ball relative ot the array of TSSPs (robot).
 */
float Tssp_system::getBallDir() {
    return ballDir;
}

/*!
 * @brief Get function for the movement speed (orbit).
 * 
 * @returns Speed from 0-255 that the robot moves at when orbiting.
 */
float Tssp_system::getMoveSpd() {
    return moveSpeed;
}

/*!
 * @brief Get function for the movement direction (orbit).
 * 
 * @returns Angle from 0-360 which the robot moves at.
 */
float Tssp_system::getMoveDir() {
    return moveDir;
}