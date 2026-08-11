/**
 * @file Vect.h
 * @brief 2D vector with dual Cartesian (i, j) and polar (mag, arg) storage.
 *
 * arg is degrees, mag is the Euclidean length. exists() is true when mag != 0.
 * Arithmetic operators primarily manipulate Cartesian components except
 * scalar * / which scale magnitude in polar form.
 */

#ifndef VECT_H_
#define VECT_H_

#include <Arduino.h>
#include "Common.h"

class Vect {
public:
    Vect();

    /**
     * @brief Construct from either polar (mag, arg) or Cartesian (i, j).
     * @param isPolar true → val1=mag, val2=arg; false → val1=i, val2=j.
     */
    Vect(float val1, float val2, bool isPolar = true);

    void setStandard(float _i, float _j); /**< Set Cartesian; recompute polar. */
    void setPolar(float _mag, float _arg); /**< Set polar; recompute Cartesian. */

    Vect operator+(Vect vector2);
    Vect operator-(Vect vector2);
    Vect operator*(float scalar);
    Vect operator/(float scalar);
    void operator+=(Vect vector2);
    void operator-=(Vect vector2);
    Vect operator*=(float scalar);
    Vect operator/=(float scalar);

    /** Magnitude comparisons (not full vector equality of components). */
    bool operator==(Vect vector2);
    bool operator!=(Vect vector2);
    bool operator<(Vect vector2);
    bool operator<=(Vect vector2);
    bool operator>(Vect vector2);
    bool operator>=(Vect vector2);

    bool exists(); /**< @return true if mag != 0 (object visible / non-zero). */
    bool isBetween(float leftAngle, float rightAngle); /**< arg inside sector. */

    float i, j;     /**< Cartesian components. */
    float mag, arg; /**< Polar magnitude and argument (deg). */

private:
    float calcI(float _mag, float _arg);
    float calcJ(float _mag, float _arg);
    float calcMag(float _i, float _j);
    float calcArg(float _i, float _j);
};

#endif
