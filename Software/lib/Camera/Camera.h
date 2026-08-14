#ifndef CAMERA_H
#define CAMERA_H

#include "Vect.h"
#include "Timer.h"

class Camera {
public:
    Camera() {}
    void init();
    void update();
    
    Vect get_ball() { return ball; };
    Vect get_attack() { return attack; };
    Vect get_defend() { return defend; };

private:
    Vect ball{0.0f, 0.0f, false};
    Vect yellow{0.0f, 0.0f, false};
    Vect blue{0.0f, 0.0f, false};
    Vect attack{0.0f, 0.0f, false};
    Vect defend{0.0f, 0.0f, false};

    Timer ballNotVis{100000};

    void read();
    void to_bearing(Vect& v);
    float px_to_mm(float mag);
};

#endif
