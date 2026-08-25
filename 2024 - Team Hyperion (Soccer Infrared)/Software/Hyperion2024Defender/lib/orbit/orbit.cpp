#include <orbit.h>
#include <common.h>

int Orbit::calculate_Direction(int ballAngle)
{
    if (ballAngle < 10 || ballAngle >= 350)
    {
        return orbitDirections[1];
    }
    else if (ballAngle >= 10 || ballAngle < 90)
    {
        return orbitDirections[2];
    }
    else if (ballAngle >= 90 || ballAngle < 135)
    {
        return orbitDirections[3];
    }
    else if (ballAngle >= 135 || ballAngle < 180)
    {
        return orbitDirections[4];
    }
    else if (ballAngle >= 180 || ballAngle < 225)
    {
        return orbitDirections[5];
    }
    else if (ballAngle >= 225 || ballAngle < 270)
    {
        return orbitDirections[6];
    }
    else if (ballAngle >= 270 || ballAngle < 350)
    {
        return orbitDirections[7];
    }
    else
    {
        return orbitDirections[8];
    }
}

int Orbit::calculate_Speed(int ballStrength)
{
    speed = (SPEED_GRADIENT)*ballStrength + SPEED_Y_INTERCEPT;
    return int(speed);
}

int Orbit::calculate_Direction2(int ballAngle2)
{
    if (ballAngle2 >= 340 && ballAngle2 < 360) {
        return ballAngle2;
    } else if (ballAngle2 <= 20 && ballAngle2 > 0) {
        return ballAngle2;
    } else if (ballAngle2 > 270 && ballAngle2 < 340) {
        return ballAngle2 - 20;
    } else if (ballAngle2 > 20 && ballAngle2 < 90) {
        return ballAngle2 + 20;
    } else if (ballAngle2 > 150 && ballAngle2 <= 180) {
        return orbitDirections[2];
    } else if (ballAngle2 > 180 && ballAngle2 < 210) {
        return orbitDirections[4];
    } else if (ballAngle2 > 210 && ballAngle2 <= 270) {
        return orbitDirections[3];
    } else if (ballAngle2 >= 90 && ballAngle2 < 150) {
        return orbitDirections[3];
    } else {
        return 0;
    }
}

int Orbit::calculate_Defense_Direction(int ballAngle3) {
    if (ballAngle3 < 360 && ballAngle3 >= 180) {
        return 270; //!!!! in main, if you are using this function, you will correct to the angle the TSSPS give.
    } else if (ballAngle3 >= 0 && ballAngle3 < 180) {
        return 90; //!!!! in main, if you are using this function, you will correct to the angle the TSSPS give.
    } else {
        return 0;
    }
}

int Orbit::calculate_Defense_Speed(int ballStrength2) {
    int speed2 = 0;
    speed2 = (NEGATIVE_SPEED_GRADIENT)*ballStrength2 + SPEED_Y_INTERCEPT;
    return int(speed2);
}

//Calculate Orbit Direction 2
//Theta = Tan(adjacent/opposite)
//Hypotenuse = Opposite/Tan(theta)
//Correct to theta, and use Hypotenuse to calculate speed.