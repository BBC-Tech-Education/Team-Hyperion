/**
 * @file Common.cpp
 * @brief Implementations of shared angular / scoring utilities.
 */

#include <Arduino.h>
#include "Common.h"

/**
 * @brief Calculates the directional angle from one angle to another clockwise.
 *
 * @param angleCounterClockwise Starting angle (deg).
 * @param angleClockwise        Target angle (deg).
 * @return Clockwise difference in [0, 360).
 */
float angle_between(float angleCounterClockwise, float angleClockwise) {
    return float_mod(angleClockwise - angleCounterClockwise, 360.0f);
}

/**
 * @brief Midpoint along the clockwise arc between two angles.
 *
 * @param angleCounterClockwise Starting angle (deg).
 * @param angleClockwise        Ending angle (deg).
 * @return Midpoint wrapped to [0, 360).
 */
float mid_angle_between(float angleCounterClockwise, float angleClockwise) {
    return float_mod(angleCounterClockwise + angle_between(angleCounterClockwise, angleClockwise) / 2.0f, 360.0f);
}

/**
 * @brief Minimum turn between two headings (unsigned).
 */
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise) {
    float angle = angle_between(angleCounterClockwise, angleClockwise);
    return fmin(angle, 360.0f - angle);
}

/**
 * @brief Map an angle into (−180, 180] for signed error terms.
 *
 * @param angle Input angle (deg), typically already in [0, 360).
 * @return Signed equivalent; angles > 180 become negative.
 */
float normaliseAngle180(float angle) {
    return (angle > 180.0 ? angle - 360.0 : angle);
}

/**
 * @brief Map an angle into [0, 360).
 *
 * @param angle Input angle (deg); negative values wrap up by +360.
 */
float normaliseAngle360(float angle) {
    return (angle > 0.0 ? angle : angle + 360.0);
}

/**
 * @brief Positive integer modulo (fixes C++ % for negative dividends).
 */
int mod(int x, int m) {
    int r = x % m;
    return r < 0 ? r + m : r;
}

/**
 * @brief Positive floating-point modulo (fixes fmod for negative dividends).
 */
float float_mod(float x, float m) {
    float r = fmod(x, m);
    return r < 0 ? r + m : r;
}

/**
 * @brief Sector membership test with wrap-around support.
 *
 * If the bound interval does not wrap (CCW < CW), use a simple open interval.
 * If it wraps (CCW > CW), the valid region is the union of (CCW, 360) U [0, CW).
 */
bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise, float angleCheck) {
    if (angleBoundCounterClockwise < angleBoundClockwise) {
        return (angleBoundCounterClockwise < angleCheck && angleCheck < angleBoundClockwise);
    } else {
        return (angleBoundCounterClockwise < angleCheck || angleCheck < angleBoundClockwise);
    }
}

/**
 * @brief Compose Bluetooth attack score from local situational features.
 *
 * Terms (approximate weights):
 *   - ballStr * 2/5
 *   - +20 if facing ball
 *   - +15 if facing goal
 *   - goalDist * 3/20
 *   - batLvl * 10/126
 */
uint8_t compute_bt_role_score(uint8_t ballStr, bool facingBall, bool facingGoal,
                              uint8_t goalDist, uint8_t batLvl) {
    uint16_t score = (ballStr * 2) / 5;
    if (facingBall) score += 20;
    if (facingGoal) score += 15;
    score += (goalDist * 3) / 20;
    score += ((uint16_t)batLvl * 10) / 126;
    return (uint8_t)(score > 255 ? 255 : score);
}
