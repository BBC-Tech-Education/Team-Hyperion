#include "camera.h"

void Camera::init() {
    Serial3.begin(115200);
}

void Camera::read() {
    while (Serial3.available() >= 5) {
        if (Serial3.read() == 254) {
            for (i = 0; i < 4; i++) {
                cameraValues[i] = Serial3.read();
            }
        }
    }
} 

//0.70833333333333333333333333333333 Degrees to analog
//1.4117647058823529411764705882353 Analog to degrees
