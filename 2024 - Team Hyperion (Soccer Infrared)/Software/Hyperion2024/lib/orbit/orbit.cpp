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

int Orbit::calculate_Speed(int ballStrength, float tsspBallAngle)
{
    ballStrength = abs(ballStrength);
    if(tsspBallAngle > 10 || tsspBallAngle >= 350) {
        return 100;
    } else {
        speed = (SPEED_GRADIENT)*ballStrength + SPEED_Y_INTERCEPT;
        if(int(speed) > 255) {
            return 255;
        } else {
            return int(speed);
        }
    }
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

int Orbit::calculate_Direction3(int ballAngle3) {
    powpow = pow(ORBIT_MULTIPLIER, 4.5*ballAngle3);
    thing = 0.04*powpow;
    ballAngleOffset = min(thing, ADD_OR_SUBTRACT_ORBIT);
    //y=min(0.04(2.71828182846^4.5*x), 60)
    // Serial.print(ballAngleOffset);
    // Serial.print(" ");
    if(ballAngle3 > 180) {
        returnVal = ballAngle3 - ballAngleOffset;
    } else {
        returnVal = ballAngle3 + ballAngleOffset;
    }
    // Serial.print(returnVal);
    // Serial.println("");
    return returnVal;
}