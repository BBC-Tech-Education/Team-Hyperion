#include "Camera.h"

void Camera::init() {
    Serial1.begin(9600);
}

void Camera::update() {
    if(Serial1.available() >= 0) {
        goal_dir = Serial1.read();
    }
}

int Camera::get_goal_dir() {
    return goal_dir;
}