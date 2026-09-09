#ifndef COMMON_H
#define COMMON_H

float angle_between(float angleCounterClockwise, float angleClockwise);
float mid_angle_between(float angleCounterClockwise, float angleClockwise);
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise);
float normaliseAngle180(float angle);
float normaliseAngle360(float angle);
float float_mod(float x, float m);
bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise, float angleCheck);
int mod(int x, int m);

#endif