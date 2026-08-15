#include <Arduino.h>
#include "Camera.h"
#include "Common.h"
#include "Config.h"
#include "Pins.h"

void Camera::init() {
    CAM_SERIAL.begin(115200);
    pinMode(GOAL_TRACK_SWITCH, INPUT);
    ballNotVis.update();
    yellowGoalNotVis.update();
    blueGoalNotVis.update();
}

void Camera::read() {
    int16_t b1 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    int16_t b2 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    Serial.print(b1);
    Serial.print("\t");
    Serial.print(b2);
    Serial.print("\t");
    if (b1 != (CAM_CENTER_X - CAM_PIXEL_SHIFT) && b2 != (CAM_CENTER_Y - CAM_PIXEL_SHIFT)) {
        yellow.setStandard(b1, b2);
        to_bearing(yellow);
        yellowGoalNotVis.update();
    }

    b1 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    b2 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    if (b1 != (CAM_CENTER_X - CAM_PIXEL_SHIFT) && b2 != (CAM_CENTER_Y - CAM_PIXEL_SHIFT)) {
        blue.setStandard(b1, b2);
        to_bearing(blue);
        blueGoalNotVis.update();
    }

    b1 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    b2 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    if (b1 != (CAM_CENTER_X - CAM_PIXEL_SHIFT) && b2 != (CAM_CENTER_Y - CAM_PIXEL_SHIFT)) {
        ball.setStandard(b1, b2);
        to_bearing(ball);
        ballNotVis.update();
    }
    Serial.print(b1);
    Serial.print("\t");
    Serial.println(b2);

    #if DEBUG_CAM_RAW
    Serial.printf("Yellow: (%.1f,%.1f)\tBlue(%.1f, %.1f)\tBall(%.1f, %.1f)\n",
                  yellow.i, yellow.j, blue.i, blue.j, ball.i, ball.j);
    #endif
}

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
    if (ballNotVis.time_has_passed_no_update()) {
        ball.setStandard(0, 0);
    }
    if(yellowGoalNotVis.time_has_passed_no_update()) {
        yellow.setStandard(0, 0);
    }
    if(blueGoalNotVis.time_has_passed_no_update()) {
        blue.setStandard(0, 0);
    }
}

float Camera::px_to_mm(float mag) {
    return 0.0000913503f * powf(mag, 3.51908) + 221.38261;
}

void Camera::to_bearing(Vect& v) {
    if (v.exists()) {
        v.arg = float_mod(270.0f - v.arg, 360.0f);
    }
}