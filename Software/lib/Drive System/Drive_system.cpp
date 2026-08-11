/**
 * @file Drive_system.cpp
 * @brief Implementation of omni drive mixing and H-bridge output.
 */

#include "Drive_system.h"
#include <math.h>

/**
 * @brief Initialise GPIO and PWM.
 *
 * Both INA/INB are driven HIGH briefly as a known idle state before PWM
 * frequency is programmed (Teensy analogWriteFrequency).
 */
void DriveSystem::init()
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        pinMode(motorInA[i], OUTPUT);
        pinMode(motorInB[i], OUTPUT);
        pinMode(motorPWM[i], OUTPUT);

        digitalWrite(motorInA[i], HIGH);
        digitalWrite(motorInB[i], HIGH);
        delayMicroseconds(100);
        analogWriteFrequency(motorPWM[i], MOTOR_ANALOG_FRQ);
    }
}

/**
 * @brief Mix translation + rotation, saturate, and write H-bridges.
 *
 * Mixer:
 *   values[i] = cos(ang + motorAng[i]) * spd + cor
 *
 * If max |values[i]| > 255, all channels are scaled by 255/max so relative
 * ratios (and thus direction) are preserved under PWM clipping.
 *
 * Direction truth table (typical dual-input driver):
 *   value > 0 → INA=1, INB=0 (forward)
 *   value < 0 → INA=0, INB=1 (reverse)
 *   value = 0 → both low after fabs round (coast/brake depends on driver)
 */
void DriveSystem::run(float spd, float ang, float cor)
{
    #if DEBUG_DRIVE_CMD
    Serial.printf("DRIVE_CMD: spd=%.2f ang=%.2f cor=%.2f\n", spd, ang, cor);
    #endif

    for (int i = 0; i < MOTOR_NUM; i++) {
        values[i] = cosf(DEG_TO_RAD * (ang + motorAng[i])) * spd + cor;
    }

    float largestSpd = 0.0f;

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        float mag = fabs(values[i]);
        largestSpd = mag > largestSpd ? mag : largestSpd;
    }

    if (largestSpd > 255.0f) {
        for (uint8_t i = 0; i < MOTOR_NUM; i++) {
            values[i] *= (255.0f / largestSpd);
        }
    }

    #if DEBUG_DRIVE_MOTORS
    Serial.printf("DRIVE_MOTORS: scale=%.2f", largestSpd > 255.0f ? 255.0f / largestSpd : 1.0f);
    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        Serial.printf(" M%d=%.1f", i, values[i]);
    }
    Serial.println();
    #endif

    for (uint8_t i = 0; i < MOTOR_NUM; i++) {
        uint8_t finalSpd = round(fabs(values[i]));
        analogWrite(motorPWM[i], finalSpd);
        digitalWrite(motorInA[i], (values[i] > 0.0f));
        digitalWrite(motorInB[i], (values[i] < 0.0f));
    }
}
