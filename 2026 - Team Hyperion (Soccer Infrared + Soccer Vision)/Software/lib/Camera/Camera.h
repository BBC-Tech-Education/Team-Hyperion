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
    Vect get_position() { return position; };

private:
    Vect ball{0.0f, 0.0f, false};
    Vect yellow{0.0f, 0.0f, false};
    Vect blue{0.0f, 0.0f, false};
    Vect attack{0.0f, 0.0f, false};
    Vect defend{0.0f, 0.0f, false};
    Vect position{0.0f, 0.0f, false};

    Timer ballNotVis{100000};
    Timer blueGoalNotVis{3000000};
    Timer yellowGoalNotVis{3000000};

    void read();
    void calculate_position();

    void px_to_mm(Vect &v);
    
};

#endif
