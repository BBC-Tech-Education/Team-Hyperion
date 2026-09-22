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
    if (b1 != (CAM_CENTER_X - CAM_PIXEL_SHIFT) && b2 != (CAM_CENTER_Y - CAM_PIXEL_SHIFT)) {
        blue.setStandard(b1, b2);
        blue = blue.to_bearing();
        blueGoalNotVis.update();
    }
    
    b1 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    b2 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    if (b1 != (CAM_CENTER_X - CAM_PIXEL_SHIFT) && b2 != (CAM_CENTER_Y - CAM_PIXEL_SHIFT)) {
        yellow.setStandard(b1, b2);
        yellow = yellow.to_bearing();
        yellowGoalNotVis.update();
    }

    b1 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    b2 = CAM_SERIAL.read() - CAM_PIXEL_SHIFT;
    if (b1 != (CAM_CENTER_X - CAM_PIXEL_SHIFT) && b2 != (CAM_CENTER_Y - CAM_PIXEL_SHIFT)) {
        ball.setStandard(b1, b2);
        ball = ball.to_bearing();
        ballNotVis.update();
    }

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
            float px = px_to_mm(blue.mag);
            float arg = blue.arg;
            blue.setPolar(px, arg);

            px = px_to_mm(yellow.mag);
            arg = yellow.arg;
            yellow.setPolar(px, arg);

            px = px_to_mm(ball.mag);
            arg = ball.arg;
            ball.setPolar(px, arg);

            if (digitalRead(GOAL_TRACK_SWITCH)) {
                attack = blue;
                defend = yellow;
            } else {
                attack = yellow;
                defend = blue;
            }
        }
    }

    // pixels to mm must happen here
    calculate_position();

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

void Camera::calculate_position() {
    if(attack.exists() && defend.exists()) {
        position = ((attack + defend) * -1.0) / 2.0;
    } else {
        Vect centerOffset(((attack.exists() ? -1.0 : 1.0) * FIELD_LENGTH_MM) / 2.0, false);
        position = centerOffset - attack;
    }
}

float Camera::px_to_mm(float mag) {
    #if CONTROL
    // old mirror px->mm
    return 0;
    #else
    // new mirror px->mm
    return 15.86018 * pow(1.0433,mag) + 50;
    #endif
}