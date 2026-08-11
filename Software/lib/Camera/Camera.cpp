/**
 * @file Camera.cpp
 * @brief OpenMV UART framing, object extraction, and goal polarity mapping.
 */

#include <Arduino.h>
#include "Camera.h"
#include "Common.h"
#include "Config.h"
#include "Pins.h"

void Camera::init() {
    CAM_SERIAL.begin(115200);
    pinMode(GOAL_TRACK_SWITCH, INPUT);
}

/**
 * @brief Parse three CAM objects: yellow goal, blue goal, ball.
 *
 * Each object is two bytes (x, y) with CAM_PIXEL_SHIFT bias removed.
 * The "not detected" sentinel from the camera is the image centre
 * (CAM_CENTER_X/Y after shift). Detected points are stored in Cartesian
 * form; Vect converts to polar (mag/arg) automatically.
 */
void Camera::read() {
    int16_t b1 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    int16_t b2 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    if (b1 != (CAM_CENTER_X - CAM_PIXEL_SHIFT) && b2 != (CAM_CENTER_Y - CAM_PIXEL_SHIFT)) {
        yellow.setStandard(b1, b2);
        to_bearing(yellow);
    } else {
        yellow.setStandard(0, 0);
    }

    b1 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    b2 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    if (b1 != (CAM_CENTER_X - CAM_PIXEL_SHIFT) && b2 != (CAM_CENTER_Y - CAM_PIXEL_SHIFT)) {
        blue.setStandard(b1, b2);
        to_bearing(blue);
    } else {
        blue.setStandard(0, 0);
    }

    b1 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    b2 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    if (b1 != (CAM_CENTER_X - CAM_PIXEL_SHIFT) && b2 != (CAM_CENTER_Y - CAM_PIXEL_SHIFT)) {
        ball.setStandard(b1, b2);
        to_bearing(ball);
    } else {
        ball.setStandard(0, 0);
    }

    #if DEBUG_CAM_RAW
    Serial.printf("Yellow: (%.1f,%.1f)\tBlue(%.1f, %.1f)\tBall(%.1f, %.1f)\n",
                  yellow.i, yellow.j, blue.i, blue.j, ball.i, ball.j);
    #endif
}

/**
 * @brief Sync on CAM_START_BYTE_1/2 then parse payloads while FIFO allows.
 *
 * GOAL_TRACK_SWITCH selects which physical colour is attack vs defend so the
 * same binary works on either field colour assignment.
 */
void Camera::update() {
    while (CAM_SERIAL.available() >= CAM_PACKET_SIZE) {
        uint8_t b1 = CAM_SERIAL.read();
        uint8_t b2 = CAM_SERIAL.peek();

        if (b1 == CAM_START_BYTE_1 && b2 == CAM_START_BYTE_2) {
            CAM_SERIAL.read();
            read();
            if (digitalRead(GOAL_TRACK_SWITCH)) {
                attack = blue;
                defend = yellow;
            } else {
                attack = yellow;
                defend = blue;
            }
        }
    }
}

/**
 * @brief Convert image-plane radius to approximate field range (mm).
 * Fit coefficients are empirical for the current lens / mounting height.
 */
float Camera::px_to_mm(float mag) {
    return 0.0000913503f * powf(mag, 3.51908) + 221.38261;
}

void Camera::to_bearing(Vect& v) {
    if (v.exists()) {
        v.arg = float_mod(270.0f - v.arg, 360.0f);
    }
}
