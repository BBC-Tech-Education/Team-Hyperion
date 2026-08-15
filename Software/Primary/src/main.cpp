#include <Arduino.h>
#include "Adafruit_BNO055.h"
#include "Bluetooth.h"
#include "Camera.h"
#include "Common.h"
#include "Config.h"
#include "Drive_system.h"
#include "Light_system.h"
#include "PID.h"
#include "PID_Autotune.h"
#include "Timer.h"
#include "Voltage_divider.h"

///////////////////////////////////// FSMs ////////////////////////////////////
enum RobotState {
    STATE_IDLE,
    STATE_GAME,
    STATE_TUNE
};

//////////////////////////////////// OBJECTS //////////////////////////////////
// ROBOT SYSTEMS
Adafruit_BNO055 bno(BNO055_SENSOR_ID, BNO055_ADDRESS_B, &Wire);
Camera cam;
DriveSystem motors;
LightSystem ls;
Bluetooth bt;

// PIDs
PID correction(KP_IMU, 0.0, KD_IMU, IMU_PID_MAX);
PID goalTrack(KP_GOALT, 0.0, KD_GOALT, GOALT_PID_MAX);
PID horizontal(KP_HOZT, 0.0, 0.0);
PID vertical(KP_VERT, 0.0, 0.0);
PID vertCam(KP_CVERT, 0.0, 0.0);
PID lineAvoid(KP_LAV, 0.0, KD_LAV, LAV_PID_MAX);
// PID localise(KP_LOC, 0.0, KD_LOC, LOC_PID_MAX);

#if PID_AUTO_TUNE
HeadingPIDAutotune headingTune;
#endif

// VOLTAGE DIVIDERS
VoltageDivider battery(ROBOT_VD_PIN, ROBOT_VOLTAGE_STABALISER, ROBOT_VOLTAGE_OFFSET);

// TIMERS
Timer batteryTimer(BATTERY_TIMER_INTERVAL);

// FINITE STATE MACHINES
RobotState state;

// EVENTS
sensors_event_t event;

// VARIABLES
float target;
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

Vect attackGoal;
Vect defendGoal;
Vect ballData;

const float SEARCH_ANGLES[4] = {45.0f, 135.0f, 225.0f, 315.0f};
uint8_t currentSearchIndex = 0; 
bool wasOnLineLastFrame = false;


/////////////////////////////////// FUNCTIONS /////////////////////////////////

/// @brief  Used to recieve data from the Secondary Teensy 4.1 over UART.
///         Updates both ball direction and strength when run.
void update_ball()
{
    while(Serial1.available() >= TSSP_PACKET_SIZE) {
        uint8_t b1 = Serial1.read();
        uint8_t b2 = Serial1.peek();
        
        if(b1 == TSSP_START_BYTE_1 && b2 == TSSP_START_BYTE_2) {
            Serial1.read();
            uint8_t ball1 = Serial1.read();
            uint8_t ball2 = Serial1.read();
            uint16_t reconstructedBall = (ball1 << 8) | ball2;
            
            relBallDir = (float)(reconstructedBall / BALL_DIR_DIVISOR);
            relBallStr = Serial1.read();

            #if DEBUG_TSSP_BALL
            Serial.printf("Dir: %.2f\tStr: %d\n", relBallDir, relBallStr);
            #endif
        }
    }
}

/// @brief  Performs calculations with the light sensor library to achieve a
///         line angle that is relative to the field rather than the robot.
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

/// @brief  Performs calculations for our attacker strategy, and runs the motors.
void calculate_attack() {
    float moveDir = 0.0f;
    float moveSpd = 0.0f;
    float moveCor = 0.0f;

    float absBallDir = float_mod(relBallDir + bearing, 360.0f);

    if (relBallStr != 0.0f) {
        wasOnLineLastFrame = false;
        float orbitTarget = target;
        if (attackGoal.exists() && GOAL_TRACKING) {
            orbitTarget = float_mod(bearing, 360.0f);
        }
        
    #if ORBIT
        float dir = normaliseAngle180(float_mod(absBallDir - orbitTarget, 360.0f));
        float ballAngDiff = (dir > 0.0f ? 1.0f : -1.0f) * fmin(90.0f, 0.000000309786f*pow(dir, 4) + 0.0000534514f*pow(dir, 3) + 0.0163822f*dir*dir - 0.00204537f*dir + 10.0f);
        float distMulti = 0.8f;
        float angleAddition = distMulti * ballAngDiff;

        #if SURGE
        surgeTimer--;
        if (((absBallDir < BALL_FRONT_MIN || absBallDir > BALL_FRONT_MAX) && (relBallStr < BALL_STR_CLOSE_THRESH))) {
            moveDir = 0.0f;
            surgeTimer = 100;
            moveSpd = SURGE_SPEED + 70.0f;
        } else if (surgeTimer > 0) {
            moveDir = 0.0f;
            moveSpd = SURGE_SPEED + 70.0f;
        } else {
            moveDir = float_mod(absBallDir + angleAddition, 360.0f);
        }
        #else
        moveDir = float_mod(absBallDir + angleAddition, 360.0f);
        #endif

        moveSpd = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0f - fabs(angleAddition / 90.0f));
    #endif

    } else {
        #if SEARCH_LEG
        moveDir = float_mod(SEARCH_ANGLES[currentSearchIndex] + bearing, 360.0f);
        moveSpd = 60.0f;
        #else
        moveDir = 0.0f; 
        moveSpd = 0.0f;
        #endif
    }
    if (absLineSize != -1.0f) {
        
        if (relBallStr == 0.0f) {
            if (!wasOnLineLastFrame) {
                currentSearchIndex += 1;
                currentSearchIndex = currentSearchIndex%3;
                wasOnLineLastFrame = true;
            }
            moveDir = float_mod(absLineAngle + 180.0f, 360.0f);
            moveSpd = -lineAvoid.update(absLineSize, -1.0f);
        } 
        
        else if (absLineSize < LINE_AVOID_THRESH) {
            if (smallestAngleBetween(moveDir, absLineAngle) < 90.0f) {
                moveSpd = sin(smallestAngleBetween(moveDir, absLineAngle) * DEG_TO_RAD) * LS_SLIDE_CONST;
                float difference = normaliseAngle180(float_mod(moveDir - absLineAngle, 360.0f));
                if (difference > 0.0f) {
                    moveDir = float_mod(absLineAngle + 90.0f, 360.0f);
                } else {
                    moveDir = float_mod(absLineAngle - 90.0f, 360.0f);
                }
            }
        } else {
            moveDir = float_mod(absLineAngle + 180.0f, 360.0f);
            moveSpd = -lineAvoid.update(absLineSize, -1.0f);
        }

    } else {
        wasOnLineLastFrame = false;
    }

    if (attackGoal.exists() && GOAL_TRACKING) {
        float goalAngle = normaliseAngle180(float_mod(attackGoal.arg, 360.0f));
        moveCor = goalTrack.update(goalAngle, 0.0f);
    } else {
        moveCor = -correction.update(normaliseAngle180(bearing), 0.0f);
    }

    motors.run(moveSpd, float_mod(moveDir - bearing, 360.0f), moveCor);
}

/// @brief  Performs calculations for our defender strategy, and runs the motors.
void calculate_defend() {
    float hoztInput = (relBallStr != 0.0f) ? -normaliseAngle180(relBallDir) : normaliseAngle180(bearing);
    float hozt = horizontal.update(hoztInput, 0.0f);
 
    float vert = 0.0f;
    if (defendGoal.exists()) {
        vert = vertCam.update(defendGoal.mag, DEFEND_CAM_TARGET);
    }
    // Serial.println(defendGoal.mag);
    float moveSpd = sqrtf(hozt*hozt + vert*vert);
    float moveDir = (atan2f(hozt, vert) * RAD_TO_DEG);
    float moveCor = 0.0f;
 
    if(defendGoal.exists()) {
        float goalAngle = float_mod(defendGoal.arg + 180.0f, 360.0f);
        moveCor = goalTrack.update(normaliseAngle180(goalAngle), 0.0f);
    } else {
        moveCor = -correction.update(normaliseAngle180(bearing), 0.0);
    }
 
    #if DEBUG_MAIN_DEFEND
    Serial.printf("Move Dir: %.2f\tMove Spd: %.2f\tMove Cor: %.2f\n", moveDir, moveSpd, moveCor);
    Serial.printf("Hozt: %.2f\tVert: %.2f\n", hozt, vert);
    #endif
    if(relBallDir > 90 && relBallDir < 270) {
        calculate_attack();
    } else {
        motors.run(moveSpd, float_mod(moveDir - bearing, 360.0f), moveCor);
    }
   
}

/// @brief  Updates our battery LED to show if our battery is low during a game.
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
#if PID_AUTO_TUNE
    state = STATE_TUNE;
    headingTune.begin();
#else
    state = STATE_IDLE;
#endif

    delay(100);
    Serial.begin(SERIAL_BAUD_RATE);
#if PID_AUTO_TUNE
    Serial.println(F("PID_AUTO_TUNE=1 - STATE_TUNE (IMU heading PD)"));
#endif

    while (!bno.begin(OPERATION_MODE_IMUPLUS)) {
        Serial.println("No BNO055 detected.");
        delay(1000);
    }
    delay(500);
    bno.setExtCrystalUse(true);
    delay(500);

    Serial1.begin(TSSP_BAUD_RATE);

    cam.init();
    motors.init();
    ls.init();

    battery.init();
    
    pinMode(ENABLE_SWITCH, INPUT);
    pinMode(COM_MODULE, INPUT);
    pinMode(PHOTOGATE_PIN, INPUT);
    pinMode(BATTERY_LED, OUTPUT);
}

void loop() {
    update_battery_led();
    #if USE_COM_MODULE
    bool motorsOn = (analogRead(COM_MODULE) > COM_MODULE_THRESH);
    #else
    bool motorsOn = digitalRead(ENABLE_SWITCH);
    #endif

    switch (state) {
        case STATE_IDLE:
            motors.run(0.0f, 0.0f, 0.0f);
            state = STATE_GAME;
            break;

        case STATE_GAME: {
            bno.getEvent(&event); 
            bearing = float_mod(event.orientation.x - target, 360.0f);

            cam.update();
            attackGoal = cam.get_attack();
            defendGoal = cam.get_defend();

            #if not OPEN
            update_ball();
            #else
            ballData = cam.get_ball();
            relBallDir = ballData.arg;
            relBallStr = ballData.mag;
            #endif
            
            ls.update();
            update_absolute_line();

            bool attack = !CONTROL;

            if (motorsOn) {
                if (attack) {
                    calculate_attack();
                } else {
                    calculate_defend();
                }
            } else {
                motors.run(0.0f, 0.0f, 0.0f);
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
            Serial.println(attackGoal.arg);
            
            break;
        }

#if PID_AUTO_TUNE
        case STATE_TUNE: {
            bno.getEvent(&event);
            bearing = float_mod(event.orientation.x - target, 360.0f);
            headingTune.update(motorsOn, bearing, motors);
            Serial.print(headingTune.kp());
            Serial.print("\t");
            Serial.println(headingTune.kd());
            #if DEBUG_MAIN_STATE
            Serial.printf("STATE_TUNE enable=%d finished=%d hasResult=%d\n",
                          motorsOn, headingTune.finished(), headingTune.hasResult());
            #endif
            break;
        }
#endif
    };
}
