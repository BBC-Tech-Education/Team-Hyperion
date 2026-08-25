/*!
 * @file Camera.cpp
 */
 
#include "Camera.h"
 
/*!
 * @brief Initalizes the Camera System for usage.
 */
void Camera::init(){
    cameraSerial.begin(115200);
}

/*!
 * @brief Main camera function to recieve the associated values from the OpenMV
 *        module.
 * 
 * @param attackBlue If the robot is attacking the blue goal or not.
 */
void Camera::update(bool attackBlue) {
    if (cameraSerial.available() >= CAM_PACKET_SIZE) {
        // read the incoming stuff
        int byte1 = cameraSerial.read();
        int byte2 = cameraSerial.peek();
        if(byte1 == CAM_START_PACK_1 && byte2 == CAM_START_PACK_2) {
            cameraSerial.read();
            int goal_x_yellow = cameraSerial.read();
            int goal_y_yellow = cameraSerial.read();
            int goal_x_blue = cameraSerial.read();
            int goal_y_blue = cameraSerial.read();
            if(goal_x_yellow != 0) {
                goal_x_yellow -= 60;
                goal_y_yellow -= 60;
            }
            if(goal_x_blue != 0) {
                goal_x_blue -= 60;
                goal_y_blue -= 60;
            }
            if (attackBlue) {
                attackGoalAngle = calculateAngleDistance(goal_y_blue, goal_x_blue);
                attackGoalDist = calcDistance(goal_x_blue, goal_y_blue);
                defendGoalAngle = calculateAngleDistance(goal_y_yellow, goal_x_yellow);
                defendGoalDist = calcDistance(goal_x_yellow, goal_y_yellow);
                seeingAttackingGoal = (goal_y_blue != 0);
                seeingDefendingGoal = (goal_y_yellow != 0);
                attackGoalX = goal_x_blue;
            } else {
                attackGoalAngle = calculateAngleDistance(goal_y_yellow, goal_x_yellow);
                attackGoalDist = calcDistance(goal_x_yellow, goal_y_yellow);
                defendGoalAngle = calculateAngleDistance(goal_y_blue, goal_x_blue);
                defendGoalDist = calcDistance(goal_x_blue, goal_y_blue);
                seeingAttackingGoal = (goal_y_yellow != 0);
                seeingDefendingGoal = (goal_y_blue != 0);
                attackGoalX = goal_y_blue;
            }
        }
    }
    #if DEBUG_CAMERA
        Serial.print("Attacking Goal: ");
        Serial.print(attackBlue);
        Serial.print(" Atk Ang: ");
        Serial.print(attackGoalAngle);
        Serial.print(" Atk Dist: ");
        Serial.print(attackGoalDist);
        Serial.print(" Def Ang: ");
        Serial.print(defendGoalAngle);
        Serial.print(" Def Dist: ");
        Serial.print(defendGoalDist);
        Serial.print(" Atk Vsb: ");
        Serial.print(seeingAttackingGoal);
        Serial.print(" Def Vsb: ");
        Serial.print(seeingDefendingGoal);
        Serial.println();
    #endif
}

/*!
 * @brief Get function for the goal angle towards the attacking goal.
 * 
 * @returns Goal angle towards the attacking goal.
 */
float Camera::getAttackGoalAngle() {
    return attackGoalAngle;
}

/*!
 * @brief Get function for the goal distance towards the attacking goal.
 * 
 * @returns Goal distance towards the attacking goal.
 */
float Camera::getAttackGoalDist() {
    return attackGoalDist;
}

/*!
 * @brief Get function for the goal angle towards the defending goal.
 * 
 * @returns Goal angle towards the defending goal.
 */
float Camera::getDefendGoalAngle() {
    return defendGoalAngle;
}

/*!
 * @brief Get function for the goal distance towards the defending goal.
 * 
 * @returns Goal distance towards the defending goal.
 */
float Camera::getDefendGoalDist() {
    return defendGoalDist;
}

/*!
 * @brief Get function for the attack correction
 * 
 * @returns speed at which the robot must turn (Attacker)
 */
float Camera::getAttackCorrection() {
    return attackCorrection;
}

/*!
 * @brief Get function for the defend correction
 * 
 * @returns speed at which the robot must turn (Defender)
 */
float Camera::getDefendCorrection() {
    return defendCorrection;
}

/*!
 * @brief Get function for checking the visibility of the attacking goal.
 * 
 * @returns If the attacking goal is visible or not.
 */
bool Camera::getAttackGoalVisible() {
    return seeingAttackingGoal;
}

/*!
 * @brief Get function for checking the visibility of the defending goal.
 * 
 * @returns If the defending goal is visible or not.
 */
bool Camera::getDefendGoalVisible() {
    return seeingDefendingGoal;
}

/*!
 * @brief Calculates the distance between two points.
 * 
 * @returns The distance between two points.
 */
float Camera::calcDistance(float x, float y){
    return sqrtf(pow(x,2) + pow(y,2));
}

/*!
 * @brief Calculates the angle between two distances.
 * @returns The angle between the two distances.
 */
float Camera::calculateAngleDistance(float opp, float adj){
    return floatMod(90 - (atan2(opp, adj) * RAD_TO_DEG), 360);
}

int Camera::getAttackGoalX() {
    return attackGoalX;
}

void Camera::debugBytes() {
    while(cameraSerial.available()) {
        Serial.print(cameraSerial.read());
        Serial.print("\t");
    }
    Serial.println();
}