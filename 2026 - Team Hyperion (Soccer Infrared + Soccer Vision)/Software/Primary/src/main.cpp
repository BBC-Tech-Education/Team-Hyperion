#include <Arduino.h>
#include "Adafruit_BNO055.h"
#include "Ball_handling.h"
#include "Bluetooth.h"
#include "Camera.h"
#include "Common.h"
#include "Config.h"
#include "Drive_system.h"
#include "Light_system.h"
#include "PID.h"
#include "Timer.h"
#include "Voltage_divider.h"
 
///////////////////////////////////// FSMs ////////////////////////////////////
enum RobotState {
    STATE_IDLE,
    STATE_CALIBRATE,
    STATE_GAME
};

//////////////////////////////////// OBJECTS //////////////////////////////////
// ROBOT SYSTEMS
Adafruit_BNO055 bno(BNO055_SENSOR_ID, BNO055_ADDRESS_B, &Wire);
BallHandling ballHandler;
Camera cam;
DriveSystem motors;
LightSystem ls;
Bluetooth bt;

// PIDs
PID correction(KP_IMU, 0.0, KD_IMU, IMU_PID_MAX);
PID goalTrackAttack(KP_GOALT_ATK, 0.0, KD_GOALT_ATK, GOALT_PID_MAX);
PID goalTrackDefend(KP_GOALT_DEF, 0.0, KD_GOALT_DEF, GOALT_PID_MAX);
PID horizontal(KP_HOZT, 0.0, 0.0);
PID vertCam(KP_CVERT, 0.0, KD_CVERT);
PID lineAvoid(KP_LAV, 0.0, KD_LAV, LAV_PID_MAX);
PID localise(KP_LOC, 0.0, KD_LOC, LOC_PID_MAX);
// VOLTAGE DIVIDERS
VoltageDivider battery(ROBOT_VD_PIN, ROBOT_VOLTAGE_STABALISER, ROBOT_VOLTAGE_OFFSET);
 
// TIMERS
Timer batteryTimer(BATTERY_TIMER_INTERVAL);
Timer localiseTimer(LOCALISE_TIMER_INTERVAL);
 
// FINITE STATE MACHINES
RobotState state;
 
// EVENTS
sensors_event_t event;
 
// VARIABLES
float target = 0.0f;
float bearing;
 
float relBallDir = 0.0f;
float relBallStr = 0;
int surgeTimer = -1;
 
float relLineAngle = -1.0f;
float relLineSize = -1.0f;
bool onField = true;
float absLineAngle = -1.0f;
float absLineSize = -1.0f;
 
float batLvl = 0.0f;
 
bool lastMotorsOn = false;
 
Vect attackGoal;
Vect defendGoal;
Vect ballData;
Vect fieldPosition;
Vect otherFieldPosition;
Vect otherBallData;
 
 
/////////////////////////////////// FUNCTIONS /////////////////////////////////

//! @brief Make a vector relative to the field rather than the robot
//! @param v Vector that is being converted
Vect absoluteVector(Vect v) {
    if(v.exists()) {
        return Vect(v.mag, float_mod(bearing + v.arg, 360.0f), true);
    } else {
        return Vect(0.0f, 0.0f, true);
    }
}
 
//! @brief Update field position vector based on goal vectors
void update_field_vectors() {
    otherFieldPosition = bt.get_other_pos();
    otherBallData = bt.get_other_ball();

    // Convert all camera vectors relative to field
    Vect globalAttack = absoluteVector(attackGoal);
    Vect globalDefend = absoluteVector(defendGoal);
    Vect globalBall = absoluteVector(ballData);

    // The vector of the goals relative to the center of the field
    Vect attackGoalPos(FIELD_LENGTH_MM / 2.0f, 0.0f, true);
    Vect defendGoalPos(-FIELD_LENGTH_MM / 2.0f, 0.0f, true);

    if (attackGoal.exists() && defendGoal.exists()) {
        // If the attack goal and defend goal exist
        Vect posFromAttack = attackGoalPos - globalAttack;
        Vect posFromDefend = defendGoalPos - globalDefend;
        fieldPosition = (posFromAttack + posFromDefend) / 2.0f;
    } else if (attackGoal.exists()) {
        // If only the attack goal exists
        fieldPosition = attackGoalPos - globalAttack;
    } else if (defendGoal.exists()) {
        // If only the defend goal exists
        Vect tempGoal = Vect(globalDefend.mag, float_mod(globalDefend.arg + 180.0f, 360.0f), true);
        fieldPosition = tempGoal + defendGoalPos;
    } else {
        // If no goals exist
        fieldPosition = Vect(0.0f, 0.0f, false);
    }
}
 
//! @brief Update the line direction and line size relative to that of the field
void update_absolute_line() {
    relLineAngle = ls.get_line_angle();
    relLineSize = ls.get_line_size();
    bool noLine = (relLineAngle == -1.0f);
    float lineDirection = noLine ? -1.0f : float_mod(relLineAngle + bearing, 360.0f);
 
    if (onField) {
        if (!noLine) {
            absLineAngle = lineDirection;
            absLineSize = relLineSize;
            onField = false;
        }
    } else {
        if (absLineSize == LINE_OUTSIDE_SIZE) {
            if (lineDirection != -1.0f && smallestAngleBetween(lineDirection, absLineAngle) >= LINE_TOUCH_REENTRY_ANGLE) {
                absLineAngle = float_mod(lineDirection + 180.0f, 360.0f);
                absLineSize = 2.0f - relLineSize;
            }
        } else {
            if (noLine) {
                if (absLineSize <= LINE_INSIDE_THRESH) {
                    onField = true;
                    absLineAngle = -1.0f;
                    absLineSize = -1.0f;
                } else {
                    absLineSize = LINE_OUTSIDE_SIZE;
                }
            } else {
                if (smallestAngleBetween(lineDirection, absLineAngle) <= 90.0f) { 
                    absLineAngle = lineDirection;
                    absLineSize = relLineSize;
                } else {
                    absLineAngle = float_mod(lineDirection + 180.0f, 360.0f);
                    absLineSize = 2.0f - relLineSize;
                }
            }
        }
    }
}
 
//! @brief Adjusts the movement direction and speed if the line is seen
//! @param mDir The current movement direction of the robot.
//! @param mSpd The current movement speed of the robot.
void line_avoid(float &mDir, float &mSpd) {
    #if LIGHT_SENSORS
    if (absLineSize != -1.0f) {
        // If the line is detected
        
        if (relBallStr == 0.0f) {
            // If we don't see the ball, then simply move in the opposit edirection of the line
            mDir = float_mod(absLineAngle + 180.0f, 360.0f);
            mSpd = -lineAvoid.update(absLineSize, -1.0f);
        } else if (absLineSize < LINE_AVOID_THRESH) {
            // If the robot is moving towards the line, and it is not too far past, stay onthe line
            if (smallestAngleBetween(mDir, absLineAngle) < 90.0f) {
                mSpd = sin(smallestAngleBetween(mDir, absLineAngle) * DEG_TO_RAD) * LS_SLIDE_CONST;
                float difference = normaliseAngle180(float_mod(mDir - absLineAngle, 360.0f));
                if (difference > 0.0f) {
                    mDir = float_mod(absLineAngle + 90.0f, 360.0f);
                } else {
                    mDir = float_mod(absLineAngle - 90.0f, 360.0f);
                }
            }
        } else {
            // The robot should move away from the line if it sees the ball but is way over
            mDir = float_mod(absLineAngle + 180.0f, 360.0f);
            mSpd = -lineAvoid.update(absLineSize, -1.0f);
        }

    }
    #endif
}
 
//! @brief Orbit algorithm, calculates how we actually move behind the ball.
//! @param mDir The current movement direction of the robot.
//! @param mSpd The current movement speed of the robot.
void orbit(float &mDir, float &mSpd) {
    float absBallDir = float_mod(relBallDir + bearing, 360.0f);
    float orbitTarget = 0.0f;

    // Adjust the "target" based on where the goal is
    if (attackGoal.exists() && GOAL_TRACKING) {
        orbitTarget = float_mod(bearing, 360.0f);
    }

    // Algorithm deciding how much "angle" to add to the current ball direction
    #if ORBIT
    #if CONTROL
    float dir = normaliseAngle180(float_mod(absBallDir - orbitTarget, 360.0f));
    float ballAngDiff = (dir > 0.0f ? 1.0f : -1.0f) * fmin(90.0f, -0.000000264461f*pow(dir, 4) + 0.0000136593f*pow(dir, 3) + 0.0183734f*dir*dir - 0.129586f*dir + 10.0f);
    float distMulti = 0.6f;
    float angleAddition = distMulti * ballAngDiff;
    #else
    float dir = normaliseAngle180(float_mod(absBallDir - orbitTarget, 360.0f));
    float ballAngDiff = (dir > 0.0f ? 1.0f : -1.0f) * fmin(90.0f, 0.000000309786f*pow(dir, 4) + 0.0000534514f*pow(dir, 3) + 0.0163822f*dir*dir - 0.00204537f*dir + 10.0f);
    float distMulti = 0.8f;
    float angleAddition = distMulti * ballAngDiff;
    #endif

    // Move forward at full speed if the robot sees the ball infront of it, close enough, or if the light gate is triggered
    #if SURGE
    if((((absBallDir < BALL_FRONT_MIN && absBallDir > BALL_FRONT_MAX) && (relBallStr < BALL_STR_CLOSE_THRESH)) || ballHandler.photogate_triggered())) {
        mDir = 0.0f;
        mSpd = SURGE_SPEED;
    } else {
        mDir = float_mod(absBallDir + angleAddition, 360.0f);
        mSpd = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0f - fabs(angleAddition / 90.0f));
    }
    #else
    mDir = float_mod(absBallDir + angleAddition, 360.0f);
    #endif
    #endif
}

//! @brief Attack code algorithm - scores goals
void run_attack() {
    float moveDir = 0.0f;
    float moveSpd = 0.0f;
    float moveCor = 0.0f;
 
    if (relBallStr != 0.0f || ballHandler.photogate_triggered()) {
        // If we can see the ball, orbit
        localiseTimer.update();
        orbit(moveDir, moveSpd);
    } else {
        // If we haven't seen the ball for a certain period of time, we can localise
        #if LOCALISATION
        if (localiseTimer.time_has_passed_no_update()) {
            moveDir = float_mod(fieldPosition.arg + 180.0f, 360.0f);
            moveSpd = fabs(localise.update(fieldPosition.mag, 0.0f));
        } else {
            moveDir = 0.0f;
            moveSpd = 0.0f;
        }
        #else
        moveDir = 0.0f;
        moveSpd = 0.0f;
        #endif
    }

    line_avoid(moveDir, moveSpd);

    if (onField) {
        float facingError = (attackGoal.exists() && GOAL_TRACKING)
            ? fabsf(normaliseAngle180(float_mod(attackGoal.arg, 360.0f)))
            : fabsf(normaliseAngle180(bearing));
        if (facingError <= 15.0f) {
            // We are facing the goal
            if(attackGoal.mag > GOAL_DIST_FOR_KICKER_ENABLE) {
                // If we are close enough to the attack goal
                if(ballData.exists()) {
                    // If the ball data exists and the ball is infront, kick
                    if((ballData.arg < BALL_FRONT_MIN && ballData.arg > BALL_FRONT_MAX)) {
                        ballHandler.kick();
                    }
                } else {
                    // If the ball data does not exist, kick
                    ballHandler.kick();
                }
            }
        }
    }
 
    // Goal Track
    if (attackGoal.exists() && GOAL_TRACKING) {
        float goalAngle = normaliseAngle180(float_mod(attackGoal.arg, 360.0f));
        moveCor = goalTrackAttack.update(goalAngle, 0.0f);
    } else {
        moveCor = -correction.update(normaliseAngle180(bearing), 0.0f);
    }
 
    motors.run(moveSpd, float_mod(moveDir - bearing, 360.0f), moveCor);
}

//! @brief Defend code algorithm - helps concede less goals
void run_defend() {
    float moveDir = 0.0f;
    float moveSpd = 0.0f;
    float moveCor = 0.0f;
    Serial.println(relBallDir);
    bool ballBehind = (relBallDir > 90.0f && relBallDir < 270.0f) && ballData.exists();
    float bearingCor = -correction.update(normaliseAngle180(bearing), 0.0);

    // Serial.println(defendGoal.mag);
    if ((relBallDir < 10.0f || relBallDir > 350.0f) && relBallStr <= 200){
        moveCor = -correction.update(normaliseAngle180(bearing), 0.0f);
        motors.run(100, relBallDir, moveCor);
    } else if(ballBehind) {
        // Orbit and face forward if ball behind
        orbit(moveDir, moveSpd);
        moveCor = bearingCor;
        motors.run(moveSpd, float_mod(moveDir - bearing, 360.0f), moveCor);
    } else {
        if(defendGoal.exists()) {
            // Rotational PID, face towards the goal
            float goalAngle = float_mod(defendGoal.arg + 180.0f, 360.0f);
            moveCor = goalTrackDefend.update(normaliseAngle180(goalAngle), 0.0f);
            // Vertical PID, stay optimal distance away from goal
            float vert = vertCam.update(defendGoal.mag, DEFEND_CAM_TARGET);
            // Hotizontal PID, move towards ball
            // If the ball does not exist, center to the middle of the field
            float hozt = horizontal.update((relBallStr != 0.0f) ? -normaliseAngle180(relBallDir) : normaliseAngle180(bearing), 0.0f);
            //hozt = 0.0f;
            // float hozt = horizontal.update(-normaliseAngle180(relBallDir), 0.0f);
            // Kick when the ball is in the capture zone
            if(onField) {
                ballHandler.kick();
            }
            Serial.println(hozt);
            Serial.println(vert);
            moveSpd = sqrtf(hozt*hozt + vert*vert);
            moveDir = (atan2f(hozt, vert) * RAD_TO_DEG);
        } else {
            // If we cannot see the defend goal, move backwards
            moveSpd = 70.0f;
            moveDir = 180.0f;
        }
    }

    line_avoid(moveDir, moveSpd);
    motors.run(moveSpd, moveDir, moveCor);
}
 
//! @brief Turns battery LED on when robot battery becomes low
void update_battery_led() {
    batLvl = battery.get_lvl();
 
    if (batLvl > ROBOT_REQUIRED_VOLT) {
        batteryTimer.update();
        digitalWrite(BATTERY_LED, LOW);
    } else if (batteryTimer.time_has_passed_no_update()) {
        digitalWrite(BATTERY_LED, HIGH);
    } else {
        digitalWrite(BATTERY_LED, LOW);
    }
}

void setup() {
    state = STATE_IDLE;

    // Initialise all components
    while (!bno.begin(OPERATION_MODE_IMUPLUS)) {
        Serial.println("No BNO055 detected.");
        delay(1000);
    }
 
    cam.init();
    motors.init();
    ls.init();
 
    bt.init();
    battery.init();
 
    ballHandler.init();
    pinMode(ENABLE_SWITCH, INPUT);
    pinMode(BATTERY_LED, OUTPUT);
}
 
void loop() {
    update_battery_led();
    bool motorsOn = digitalRead(ENABLE_SWITCH);

    // Decide state based on enable switch.
    if (!motorsOn) {
        state = STATE_IDLE;
    } else if (!lastMotorsOn) {
        state = STATE_CALIBRATE;
    }
 
    switch (state) {
        case STATE_IDLE:
            // Stop running motors
            motors.run(0.0f, 0.0f, 0.0f);
            break;
 
        case STATE_CALIBRATE:
            // Reset compass heading
            bno.getEvent(&event);
            target = event.orientation.x;
            state = STATE_GAME;
            break;
 
        case STATE_GAME: {
            // Update compass heading
            bno.getEvent(&event); 
            bearing = float_mod(event.orientation.x - target, 360.0f);
 
            // Update camera data
            cam.update();
            attackGoal = cam.get_attack();
            defendGoal = cam.get_defend();
            ballData = cam.get_ball();
            relBallDir = ballData.arg;
            relBallStr = ballData.mag;
            update_field_vectors();
 
            // Update light sensor board data
            ls.update();
            update_absolute_line();
 
            // Update robot logic based on bluetooth
            if (false) {
                run_attack();
            } else {
                run_defend();
            }
            break;
        }
    };
 
    ballHandler.update(relBallStr);
    bt.update(motorsOn, ballData, fieldPosition, ballHandler.kicker_ready());
    lastMotorsOn = motorsOn;
}