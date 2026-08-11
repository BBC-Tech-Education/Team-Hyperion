/**
 * @file Camera.h
 * @brief UART vision front-end for OpenMV (goals + optional ball).
 *
 * Packet (after dual start bytes): three XY pixel pairs for yellow, blue, ball.
 * Goal colour assignment is selected by GOAL_TRACK_SWITCH (attack/defend swap).
 */

#ifndef CAMERA_H
#define CAMERA_H

#include "Vect.h"

class Camera {
public:
    Camera() {}

    /** Open camera UART and goal-track polarity switch GPIO. */
    void init();

    /**
     * @brief Drain UART, parse complete frames, update attack/defend/ball.
     * Safe to call every control loop iteration.
     */
    void update();

    Vect get_ball()   { return ball; }
    Vect get_attack() { return attack; }
    Vect get_defend() { return defend; }

private:
    Vect ball{0.0f, 0.0f, false};
    Vect yellow{0.0f, 0.0f, false};
    Vect blue{0.0f, 0.0f, false};
    Vect attack{0.0f, 0.0f, false}; /**< Opponent goal after switch mapping. */
    Vect defend{0.0f, 0.0f, false}; /**< Own goal after switch mapping. */

    /** Consume one payload (yellow, blue, ball XY) already past sync. */
    void read();

    /**
     * @brief Rotate camera polar argument into robot forward frame.
     * Maps vision math frame via: arg = (270 - arg) mod 360.
     */
    void to_bearing(Vect& v);

    /**
     * @brief Empirical pixel-radius → millimetre range model (cubic fit).
     * @param mag Pixel distance from image centre.
     * @return Approximate range in mm.
     */
    float px_to_mm(float mag);
};

#endif
