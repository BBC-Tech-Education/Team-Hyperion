#ifndef CAMERA_H
#define CAMERA_H
#include <Arduino.h>

class Camera {
public:
    Camera() {};
    void init();
    void read();
private:
    int cameraValues[4];
    int i;
};
#endif