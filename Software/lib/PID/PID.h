/**
 * @file PID.h
 * @brief Discrete-time PID controller for heading / positioning loops.
 *
 * Output = Kp*e + Ki*∫e dt − Kd*(de/dt) with optional symmetric clamp.
 * Timebase is micros(); suitable for variable-rate Arduino loop() calls.
 */

#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
public:
    /**
     * @param p           Proportional gain.
     * @param i           Integral gain.
     * @param d           Derivative gain.
     * @param absoluteMax If non-zero, clamp output to ±absoluteMax.
     */
    PID(float p, float i, float d, float absoluteMax = 0.0);

    /**
     * @brief Compute one control step.
     * @param input    Measured process value.
     * @param setpoint Desired value.
     * @return Control effort (clamped if absMax != 0).
     */
    float update(float input, float setpoint);

private:
    float kp;
    float ki;
    float kd;
    uint32_t lastTime;   /**< micros() stamp of previous update. */
    float lastError = 0;
    float absMax;        /**< 0 ⇒ no saturation. */
    float integral;      /**< Accumulated ∫error dt. */
};

#endif
