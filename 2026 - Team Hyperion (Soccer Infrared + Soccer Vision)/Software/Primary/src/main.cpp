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
PID vertCam(KP_CVERT, 0.0, 0.0);
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
 
void update_field_vectors() {
    otherFieldPosition = bt.get_other_pos();
    otherBallData = bt.get_other_ball();

    if (attackGoal.exists() && defendGoal.exists()) {
        fieldPosition = ((attackGoal + defendGoal) * -1.0) / 2.0;
    } else if (attackGoal.exists() || defendGoal.exists()) {
        Vect centerOffset(((attackGoal.exists() ? -1.0 : 1.0) * FIELD_LENGTH_MM) / 2.0, false);
        fieldPosition = centerOffset - attackGoal;
    } else {
        fieldPosition = Vect(0.0f, 0.0f, false);
    }

    bool hasRobotPos = fieldPosition.exists();
    bool hasRobotBall = ballData.exists();
    bool hasOtherPos = otherFieldPosition.exists();
    bool hasOtherBall = otherBallData.exists();
    int knownCount = (int)hasRobotPos + (int)hasRobotBall + (int)hasOtherPos + (int)hasOtherBall;

    if (knownCount == 3) {
        if (!hasRobotPos) {
            fieldPosition = otherFieldPosition + otherBallData - ballData;
        } else if (!hasRobotBall) {
            ballData = otherFieldPosition + otherBallData - fieldPosition;
        } else if (!hasOtherPos) {
            otherFieldPosition = fieldPosition + ballData - otherBallData;
        } else {
            otherBallData = fieldPosition + ballData - otherFieldPosition;
        }
    } else if (!fieldPosition.exists()) {
        fieldPosition = Vect(0.0f, 0.0f, false);
    }
}
 
Vect move_to(Vect targetPosition) {
    return targetPosition - fieldPosition;
}
 
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
 
void line_avoid(float &mDir, float &mSpd) {
    #if LIGHT_SENSORS
    if (absLineSize != -1.0f) {
        
        if (relBallStr == 0.0f) {
            mDir = float_mod(absLineAngle + 180.0f, 360.0f);
            mSpd = -lineAvoid.update(absLineSize, -1.0f);
        } 
        
        else if (absLineSize < LINE_AVOID_THRESH) {
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
            mDir = float_mod(absLineAngle + 180.0f, 360.0f);
            mSpd = -lineAvoid.update(absLineSize, -1.0f);
        }

    }
    #endif
}
 
void orbit(float &mDir, float &mSpd) {
    float absBallDir = float_mod(relBallDir + bearing, 360.0f);
    float orbitTarget = 0.0f;
    if (attackGoal.exists() && GOAL_TRACKING) {
        orbitTarget = float_mod(bearing, 360.0f);
    }
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
 
void run_attack() {
    float moveDir = 0.0f;
    float moveSpd = 0.0f;
    float moveCor = 0.0f;
 
    if (relBallStr != 0.0f || ballHandler.photogate_triggered()) {
        localiseTimer.update();
        orbit(moveDir, moveSpd);
    } else {
        #if LOCALISATION
        if (localiseTimer.time_has_passed_no_update()) {
            Vect targetVector(0.0f, 0.0f, false);
            Vect moveVector = move_to(targetVector);
            moveVector = moveVector.to_bearing();
            moveDir = moveVector.arg;
            moveSpd = fabs(localise.update(moveVector.mag, 0.0f));
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
            if(attackGoal.mag > GOAL_DIST_FOR_KICKER_ENABLE) {
                if(ballData.exists()) {
                    if((ballData.arg < BALL_FRONT_MIN && ballData.arg > BALL_FRONT_MAX)) {
                        ballHandler.kick();
                    }
                } else {
                    ballHandler.kick();
                }
            }
        }
    }
 
    if (attackGoal.exists() && GOAL_TRACKING) {
        float goalAngle = normaliseAngle180(float_mod(attackGoal.arg, 360.0f));
        moveCor = goalTrackAttack.update(goalAngle, 0.0f);
    } else {
        moveCor = -correction.update(normaliseAngle180(bearing), 0.0f);
    }
 
    motors.run(moveSpd, float_mod(moveDir - bearing, 360.0f), moveCor);
}
 
void run_defend() {
    float moveDir = 0.0f;
    float moveSpd = 0.0f;
    float moveCor = 0.0f;
    bool ballBehind = relBallDir > 90.0f && relBallDir < 270.0f;
    float bearingCor = -correction.update(normaliseAngle180(bearing), 0.0);
 
    if(ballBehind) {
        orbit(moveDir, moveSpd);
        moveCor = bearingCor;
        motors.run(moveSpd, float_mod(moveDir - bearing, 360.0f), moveCor);
    } else {
        if(defendGoal.exists()) {
            float goalAngle = float_mod(defendGoal.arg + 180.0f, 360.0f);
            //moveCor = goalTrackDefend.update(normaliseAngle180(goalAngle), 0.0f);
            moveCor = bearingCor;
            float vert = vertCam.update(defendGoal.mag / 4.0f, DEFEND_CAM_TARGET);
            float hozt = 0.0f;
            if(relBallStr != 0.0f) {
                hozt = horizontal.update((relBallStr != 0.0f) ? -normaliseAngle180(relBallDir) : normaliseAngle180(bearing), 0.0f);
            }
            if((relBallDir < BALL_FRONT_MIN || relBallDir > BALL_FRONT_MAX) && (relBallStr < BALL_STR_CLOSE_THRESH && relBallStr != 0.0f)) {
                moveDir = 0.0f;
                moveSpd = SURGE_SPEED + 70.0f;
                if(onField) {
                    ballHandler.kick();
                }
            } else {
                moveSpd = sqrtf(hozt*hozt + vert*vert);
                moveDir = (atan2f(hozt, vert) * RAD_TO_DEG);
            }
        } else {
            moveSpd = 70.0f;
            moveDir = 180.0f;
        }
        motors.run(moveSpd, moveDir, moveCor);
    }
    
    line_avoid(moveDir, moveSpd);
}
 
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
 
    if (!motorsOn) {
        state = STATE_IDLE;
    } else if (!lastMotorsOn) {
        state = STATE_CALIBRATE;
    }
 
    switch (state) {
        case STATE_IDLE:
            motors.run(0.0f, 0.0f, 0.0f);
            break;
 
        case STATE_CALIBRATE:
            bno.getEvent(&event);
            target = event.orientation.x;
            state = STATE_GAME;
            break;
 
        case STATE_GAME: {
            bno.getEvent(&event); 
            bearing = float_mod(event.orientation.x - target, 360.0f);
 
            cam.update();
            attackGoal = cam.get_attack();
            defendGoal = cam.get_defend();
            ballData = cam.get_ball();

            
            update_field_vectors();
            relBallDir = ballData.arg;
            relBallStr = ballData.mag;
 
            ls.update();
            update_absolute_line();
 
            if (true) {
                run_attack();
            } else {
                run_defend();
            }
 
            #if DEBUG_MAIN_IMU
            Serial.printf("Bearing: %.2f\tRaw: %.2f\n", bearing, event.orientation.x);
            #endif
 
            #if DEBUG_MAIN_GOALS
            Serial.printf("Attack Ang: %.2f\tAttack Dist: %.2f\tAttack Vis: %d\n",
                          attackGoal.arg, attackGoal.mag, attackGoal.exists());
            Serial.printf("Defend Ang: %.2f\tDefend Dist: %.2f\tDefend Vis: %d\n",
                          defendGoal.arg, defendGoal.mag, defendGoal.exists());
            #endif
 
            #if DEBUG_MAIN_LINE
            Serial.printf("Rel Ang: %.2f\tRel Size: %.2f\n", relLineAngle, relLineSize);
            Serial.printf("Abs Ang: %.2f\tAbs Size: %.2f\tOn Field: %d\n",
                          absLineAngle, absLineSize, onField);
            #endif
 
            #if DEBUG_MAIN
            Serial.println();
            #endif
            break;
        }
    };
 
    ballHandler.update(relBallStr);
    bt.update(motorsOn, ballData, fieldPosition);
    lastMotorsOn = motorsOn;
}