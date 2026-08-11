/**
 * @file Drive_system.h
 * @brief Omni-wheel drive mixer and H-bridge PWM driver (Primary MCU).
 *
 * @details
 * Models four motors at 45° / 135° / 225° / 315° body angles. Translation is
 * projected onto each wheel with cos(commandAngle + motorAngle), then a shared
 * rotational correction term is added. Outputs are magnitude-scaled to fit
 * within 8-bit PWM (0..255) before direction pins are set.
 */

#ifndef DRIVE_SYSTEM_H
#define DRIVE_SYSTEM_H

#include <Arduino.h>
#include "Config.h"
#include "Pins.h"

class DriveSystem {
public:
    DriveSystem() {};

    /** Configure direction/PWM pins and PWM carrier frequency. */
    void init();

    /**
     * @brief Command holonomic motion.
     * @param spd Translation magnitude (PWM units; may exceed 255 before scale).
     * @param ang Translation direction in robot body frame (deg).
     * @param cor Rotational bias added equally to all motors (signed PWM units).
     */
    void run(float spd, float ang, float cor);

private:
    uint8_t motorInA[MOTOR_NUM] = {FRINA, FLINA, BLINA, BRINA}; /**< H-bridge A. */
    uint8_t motorInB[MOTOR_NUM] = {FRINB, FLINB, BLINB, BRINB}; /**< H-bridge B. */
    uint8_t motorPWM[MOTOR_NUM] = {FRPWM, FLPWM, BLPWM, BRPWM}; /**< Speed PWM. */
    float motorAng[MOTOR_NUM] = {45.0, 135.0, 225.0, 315.0};    /**< Wheel axes (deg). */
    float values[MOTOR_NUM] = {0.0};                            /**< Signed mixer output. */
};

#endif
