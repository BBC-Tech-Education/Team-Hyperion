/**
 * @file Common.h
 * @brief Shared angle utilities and Bluetooth role scoring helpers.
 *
 * All angle helpers operate in degrees unless noted. Prefer float_mod over
 * fmod when a non-negative remainder in [0, m) is required.
 */

#ifndef COMMON_H
#define COMMON_H

/** Clockwise arc length from @p angleCounterClockwise to @p angleClockwise, [0, 360). */
float angle_between(float angleCounterClockwise, float angleClockwise);

/** Midpoint of the clockwise arc between two angles, wrapped to [0, 360). */
float mid_angle_between(float angleCounterClockwise, float angleClockwise);

/** Smallest unsigned angular separation between two headings, [0, 180]. */
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise);

/** Wrap to (−180, 180] for signed heading / PID error. */
float normaliseAngle180(float angle);

/** Wrap to [0, 360) for absolute headings. */
float normaliseAngle360(float angle);

/** Floating modulo with non-negative remainder in [0, m). */
float float_mod(float x, float m);

/**
 * @brief True if @p angleCheck lies strictly inside the clockwise sector
 *        from @p angleBoundCounterClockwise to @p angleBoundClockwise
 *        (handles wrap across 0°).
 */
bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise, float angleCheck);

/** Integer modulo with non-negative remainder in [0, m). */
int mod(int x, int m);

/**
 * @brief Heuristic attack-suitability score for Bluetooth role negotiation.
 *
 * Weighted mix of ball strength, facing flags, goal distance, and battery.
 * Result is saturated to uint8 [0, 255].
 */
uint8_t compute_bt_role_score(uint8_t ballStr, bool facingBall, bool facingGoal,
                              uint8_t goalDist, uint8_t batLvl);

#endif
