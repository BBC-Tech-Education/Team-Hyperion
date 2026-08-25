#ifndef CAMERA_H
#define CAMERA_H

#include <Arduino.h>

class Camera {
public:
    Camera();
    void init();
    void update();
    int get_goal_dir();
private:
    int goal_dir;
};
#endif