/*!
 * @file LSLB.h
 *
 * @mainpage This is a library to read what the camera is transmitting for goal 
 * tracking
 * 
 * @date 30/04/25
 *
 * @author T.McCabe (Brisbane Boys' College)
*/
#ifndef CAMERA_H
#define CAMERA_H
 
#include <Arduino.h>
#include <PID.h>
#include <math.h>
#include <config.h>
#include <common.h>
 
/*!
 * @brief Class that stores state and functions for reading what the camera is 
          transmitting
*/
class Camera {
public:
    Camera() {};
    void init();
    void update(bool attackBlue);

    float getAttackCorrection();
    float getDefendCorrection();
    float getAttackGoalAngle();
    float getAttackGoalDist();
    float getDefendGoalAngle();
    float getDefendGoalDist();
    int getAttackGoalX();
    bool getAttackGoalVisible();
    bool getDefendGoalVisible();

    void debugBytes();

private:
    float attackGoalAngle;
    float attackGoalDist;
    float defendGoalAngle;
    float defendGoalDist;
    float attackGoalX;
    float attackCorrection;
    float defendCorrection;
    bool seeingAttackingGoal;
    bool seeingDefendingGoal;

    float calcDistance(float x, float y);
    float calculateAngleDistance(float opp, float adj);
};
 
#endif