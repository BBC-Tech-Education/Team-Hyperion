/*!
 * @file common.cpp
 */

 #include "Common.h"

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
float floatMod(float x, float m) {
    float r = fmod(x, m);
    return r<0 ? r+m : r;
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
int intMod(int x, int m) {
    int r = x % m;
    return r < 0 ? r + m : r;
}

/**
 * @brief Calculates the smallest angular difference between two angles.
 *
 * This function computes the absolute difference between two angles on a circle,
 * ensuring the result is always in the range [0, 180] degrees. It accounts for the 
 * circular nature of angles, meaning that the difference between 350° and 10° is 20°, 
 * not 340°.
 *
 * @param a The first angle in degrees (typically in range [0, 360), but any float is accepted).
 * @param b The second angle in degrees (typically in range [0, 360), but any float is accepted).
 * 
 * @returns The smallest difference between the two angles, in degrees.
 */
float circularDiff(float a, float b) {
    float diff = fabs(a - b);
    return (diff <= 180) ? diff : 360 - diff;
}

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
float angleBetween(float angleCounterClockwise, float angleClockwise) {
    return floatMod(angleClockwise - angleCounterClockwise, 360);
}

/**
 * @brief Calculates the smallest angular difference between two angles.
 *
 * Returns the minimal angular distance between two angles, regardless of direction.
 * Always returns a value in the range [0, 180].
 *
 * @param angleCounterClockwise The first angle (in degrees).
 * @param angleClockwise The second angle (in degrees).
 * 
 * @returns The smallest angle between the two angles (in degrees).
 */
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise) {
    float ang = angleBetween(angleCounterClockwise, angleClockwise);
    return fmin(ang, 360 - ang);
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
float midAngleBetween(float angleCounterClockwise, float angleClockwise) {
    return floatMod(angleCounterClockwise + angleBetween(angleCounterClockwise, angleClockwise) / 2.0, 360);
}