/**
 * @file PID.cpp
 * @brief PID update using wall-clock dt from micros().
 */

#include "PID.h"

PID::PID(float p, float i, float d, float absoluteMax) {
    kp = p;
    ki = i;
    kd = d;
    absMax = absoluteMax;

    integral = 0.0f;
    lastError = 0.0f;
    lastTime = micros();
}

/**
 * @note Derivative is computed as −(e − e_prev)/dt so that with the leading
 * minus in the output sum it behaves as a standard Kd*de/dt term on error.
 * No anti-windup is applied beyond optional output clamping.
 */
float PID::update(float input, float setpoint) {
    float derivative = 0.0f;
    float error = setpoint - input;

    uint32_t currentTime = micros();
    float elapsedTime = (currentTime - lastTime) / 1000000.0f; /* seconds */
    lastTime = currentTime;

    integral += elapsedTime * error;

    derivative = -(error - lastError) / elapsedTime;

    lastError = error;
    lastTime = currentTime;

    float correction = kp * error + ki * integral - kd * derivative;

    return absMax == 0.0f ? correction : constrain(correction, -absMax, absMax);
}
