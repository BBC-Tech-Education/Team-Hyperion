#include <Arduino.h>
#include "Common.h"

/**
 * @brief Calculates the directional angle from one angle to another in a clockwise direction.
 *
 * Returns the angular distance from `angleCounterClockwise` to `angleClockwise`, measured clockwise,
 * wrapped within the range [0, 360).
 *
 * @param angleCounterClockwise The starting angle (in degrees).
 * @param angleClockwise The target angle (in degrees).
 * 
 * @returns The clockwise angle difference between the two angles (in degrees).
 */
float angle_between(float angleCounterClockwise, float angleClockwise) {
    return float_mod(angleClockwise - angleCounterClockwise, 360.0f);
}

/**
 * @brief Calculates the midpoint angle between two angles.
 *
 * Returns the angle exactly halfway between `angleCounterClockwise` and `angleClockwise`,
 * moving in a clockwise direction from `angleCounterClockwise`.
 *
 * @param angleCounterClockwise The starting angle (in degrees).
 * @param angleClockwise The ending angle (in degrees).
 * 
 * @returns The midpoint angle between the two given angles (in degrees), wrapped to [0, 360).
 */
float mid_angle_between(float angleCounterClockwise, float angleClockwise) {
    return float_mod(angleCounterClockwise + angle_between(angleCounterClockwise, angleClockwise) / 2.0f, 360.0f);
}

float smallestAngleBetween(float angleCounterClockwise, float angleClockwise) {
    float angle = angle_between(angleCounterClockwise, angleClockwise);
    return fmin(angle, 360.0f - angle);
}


/**
 * @brief Normalises an angle into the range (-180, 180].
 *
 * Converts any given angle into its equivalent within a semi-circle range. 
 * This is particularly useful for calculating steering offsets or relative 
 * headings where "left" is negative and "right" is positive.
 *
 * @param angle The input angle in degrees.
 * * @returns The normalised angle in degrees, wrapped to (-180, 180].
 */
float normaliseAngle180(float angle) {
    return (angle > 180.0 ? angle - 360.0 : angle);
}

/**
 * @brief Normalises an angle into the range [0, 360).
 *
 * Ensures the provided angle is mapped to a standard circular rotation.
 * This handles negative wraps by bringing them back into the positive 
 * 0-360 degree spectrum.
 *
 * @param angle The input angle in degrees.
 * * @returns The normalised angle in degrees, wrapped to [0, 360).
 */
float normaliseAngle360(float angle) {
    return (angle > 0.0 ? angle : angle + 360.0);
}

/**
 * @brief Computes a positive modulo result for integers.
 *
 * Returns the result of `x mod m`, ensuring the result is always in the range [0, m).
 * This handles negative `x` values correctly, unlike the standard `%` operator.
 *
 * @param x The dividend (can be negative).
 * @param m The modulus (must be positive).
 * 
 * @returns The positive remainder of `x` modulo `m`.
 */
int mod(int x, int m) {
    int r = x % m;
    return r < 0 ? r + m : r;
}

/*!
 * @brief Computes the floating-point modulus operation that always returns a 
 *        non-negative remainder.
 * 
 * This function calculates the remainder of `x` divided by `m`, ensuring that 
 * the result is always non-negative.
 * It correctly handles negative values of `x` by adding `m` when the remainder 
 * is negative.
 * 
 * @param x The dividend (floating-point number).
 * @param m The divisor (floating-point number).
 * 
 * @returns The non-negative remainder of `x` divided by `m`.
 */
float float_mod(float x, float m) {
    float r = fmod(x, m);
    return r<0 ? r+m : r;
}

bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise, float angleCheck) {
	if(angleBoundCounterClockwise < angleBoundClockwise) {
		return(angleBoundCounterClockwise < angleCheck && angleCheck < angleBoundClockwise);
	} else {
		return(angleBoundCounterClockwise < angleCheck || angleCheck < angleBoundClockwise);
	}
}

uint8_t compute_bt_role_score(uint8_t ballStr, bool facingBall, bool facingGoal,
                           uint8_t goalDist, uint8_t batLvl) {
    uint16_t score = (ballStr * 2) / 5;
    if (facingBall) score += 20;
    if (facingGoal) score += 15;
    score += (goalDist * 3) / 20;
    score += ((uint16_t)batLvl * 10) / 126;
    return (uint8_t)(score > 255 ? 255 : score);
}