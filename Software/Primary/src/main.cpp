/**
 * @file main.cpp
 * @brief Primary Teensy 4.1 application firmware — RCJ Lightweight / Open Soccer.
 *
 * @details
 * Dual-MCU architecture:
 *   - This MCU (Primary) owns drive, IMU, camera UART, lightgate line sensing,
 *     battery monitoring, and high-level attack/defend strategy.
 *   - The Secondary Teensy streams IR ball direction/strength over UART (Serial1)
 *     using a framed TSSP packet (except in OPEN mode, where the camera supplies
 *     ball data instead).
 *
 * Control loop (STATE_GAME):
 *   1. Read BNO055 heading → robot-relative bearing about a fixed field target
 *   2. Poll OpenMV / camera UART for yellow/blue goals (+ ball in OPEN)
 *   3. Poll Secondary for ball IR vector (Lightweight)
 *   4. Update light-sensor line estimate and lift it into field-absolute frame
 *   5. Run attack or defend kinematics and command the omni drive
 *
 * Angle conventions (degrees unless noted):
 *   - 0° = robot forward / field reference depending on context
 *   - Absolute angles are field-frame; relative angles are robot-frame
 *   - float_mod(..., 360) keeps values in [0, 360)
 *
 * @note Compile-time behaviour is selected via Config.h macros (ORBIT, OPEN,
 *       GOAL_TRACKING, debug flags, COM module enable, etc.).
 */

#include <Arduino.h>
#include "Adafruit_BNO055.h"
#include "Bluetooth.h"
#include "Camera.h"
#include "Common.h"
#include "Config.h"
#include "Drive_system.h"
#include "Light_system.h"
#include "PID.h"
#include "Timer.h"
#include "Voltage_divider.h"

/* -------------------------------------------------------------------------- */
/* Finite-state machine                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Top-level robot operating modes.
 *
 * STATE_IDLE  — motors forced stopped; immediately transitions to GAME.
 * STATE_GAME  — full sensor fusion + strategy execution.
 */
enum RobotState {
    STATE_IDLE,
    STATE_GAME
};

/* -------------------------------------------------------------------------- */
/* Subsystem instances                                                        */
/* -------------------------------------------------------------------------- */

/** BNO055 9-DoF IMU on Wire; IMUPLUS mode provides fused heading. */
Adafruit_BNO055 bno(BNO055_SENSOR_ID, BNO055_ADDRESS_B, &Wire);

/** OpenMV / camera UART parser (goals + optional ball). */
Camera cam;

/** Four-motor omni drive (mecanum/omni wheel mixing). */
DriveSystem motors;

/** Multiplexed reflectance array → robot-relative line angle/size. */
LightSystem ls;

/** Inter-robot Bluetooth role negotiation (instantiated; strategy may use later). */
Bluetooth bt;

/* -------------------------------------------------------------------------- */
/* PID controllers                                                            */
/* Units: gains from Config.h; outputs feed drive speed/correction channels.  */
/* -------------------------------------------------------------------------- */

/** Holds robot heading to field target when goal tracking is unavailable. */
PID correction(KP_IMU, 0.0, KD_IMU, IMU_PID_MAX);

/** Yaws robot to face attack goal (or away from defend goal). */
PID goalTrack(KP_GOALT, 0.0, KD_GOALT, GOALT_PID_MAX);

/** Defender: lateral (sidestep) component from ball/heading error. */
PID horizontal(KP_HOZT, 0.0, 0.0);

/** Defender: depth from absolute line size (stay on own half). */
PID vertical(KP_VERT, 0.0, 0.0);

/** Defender: depth from camera distance to own goal when line is unseen. */
PID vertCam(KP_CVERT, 0.0, 0.0);

/** Attacker/common: push away from white line using absolute line size. */
PID lineAvoid(KP_LAV, 0.0, KD_LAV, LAV_PID_MAX);
// PID localise(KP_LOC, 0.0, KD_LOC, LOC_PID_MAX);

/** Scaled ADC → pack voltage (V). */
VoltageDivider battery(ROBOT_VD_PIN, ROBOT_VOLTAGE_STABALISER, ROBOT_VOLTAGE_OFFSET);

/** Debounce / latch window for low-battery LED indication. */
Timer batteryTimer(BATTERY_TIMER_INTERVAL);

RobotState state;

/** Latest BNO055 orientation event (uses .orientation.x as yaw, deg). */
sensors_event_t event;

/* -------------------------------------------------------------------------- */
/* Runtime state                                                              */
/* -------------------------------------------------------------------------- */

float target;   /**< Field heading zero reference (deg), set at kickoff/calib. */
float bearing;  /**< Robot yaw relative to @ref target, wrapped to [0, 360). */

float relBallDir = 0.0f; /**< Ball angle in robot frame (deg). */
float relBallStr = 0;    /**< Ball proximity / IR strength (0 = unseen). */

float relLineAngle = -1.0f; /**< Robot-frame line direction; -1 = no line. */
float relLineSize  = -1.0f; /**< Robot-frame line penetration depth; -1 = none. */
bool  onField      = true;  /**< True while considered fully inside playable area. */
float absLineAngle = -1.0f; /**< Field-frame line direction; -1 = none. */
float absLineSize  = -1.0f; /**< Field-frame line size (may invert when outside). */

float batLvl = 0.0f; /**< Measured pack voltage (V). */

Vect attackGoal; /**< Opponent goal polar vector from camera. */
Vect defendGoal; /**< Own goal polar vector from camera. */
Vect ballData;   /**< Camera ball vector (OPEN mode only). */


/* -------------------------------------------------------------------------- */
/* Sensor / perception helpers                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Drain Secondary UART and update ball direction/strength.
 *
 * Packet format (TSSP_PACKET_SIZE bytes after sync):
 *   [0xFF][0xFF][dir_hi][dir_lo][str]
 * where direction is fixed-point: reconstructed_u16 / BALL_DIR_DIVISOR (deg).
 *
 * Sync uses read()+peek() so a lone 0xFF does not permanently desync the stream.
 * Excess bytes are consumed until the FIFO has fewer than one full packet.
 */
void update_ball()
{
    while (Serial1.available() >= TSSP_PACKET_SIZE) {
        uint8_t b1 = Serial1.read();
        uint8_t b2 = Serial1.peek();

        if (b1 == TSSP_START_BYTE_1 && b2 == TSSP_START_BYTE_2) {
            Serial1.read(); /* consume second sync byte */
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

/**
 * @brief Lift robot-relative line sensing into a field-absolute line model.
 *
 * @details
 * LightSystem reports line angle/size in the robot body frame. Adding IMU
 * bearing yields a candidate field-frame direction. A latching state machine
 * then tracks whether the robot is still on-field or has crossed outside:
 *
 *   onField == true
 *     - First sighting of white latches absLine* and clears onField.
 *
 *   onField == false
 *     - If absLineSize == LINE_OUTSIDE_SIZE (fully outside marker):
 *         Re-entry from the opposite side flips the stored angle by 180° and
 *         mirrors size (2 - relSize) so "deeper outside" still grows correctly.
 *     - Otherwise while still seeing line:
 *         Same-hemisphere updates refresh abs angle/size; opposite hemisphere
 *         is treated as the far side of the line (flip + mirror).
 *     - When line disappears:
 *         Small abs size → re-enter field and clear abs line.
 *         Large abs size → mark as fully outside (LINE_OUTSIDE_SIZE).
 *
 * Sentinels: angle/size of -1.0f mean "no line currently modelled".
 */
void update_absolute_line() {
    relLineAngle = ls.get_line_angle();
    relLineSize  = ls.get_line_size();
    bool noLine  = (relLineAngle == -1.0f);

    /* Robot-frame → field-frame; keep -1 when no detection. */
    float lineDirection = noLine ? -1.0f : float_mod(relLineAngle + bearing, 360.0f);

    if (onField) {
        if (!noLine) {
            absLineAngle = lineDirection;
            absLineSize  = relLineSize;
            onField      = false;
        }
    } else {
        if (absLineSize == LINE_OUTSIDE_SIZE) {
            /* Outside: only accept re-contact from roughly the opposite side. */
            if (lineDirection != -1.0f &&
                smallestAngleBetween(lineDirection, absLineAngle) >= LINE_TOUCH_REENTRY_ANGLE) {
                absLineAngle = float_mod(lineDirection + 180.0f, 360.0f);
                absLineSize  = 2.0f - relLineSize;
            }
        } else {
            if (noLine) {
                if (absLineSize <= LINE_INSIDE_THRESH) {
                    onField      = true;
                    absLineAngle = -1.0f;
                    absLineSize  = -1.0f;
                } else {
                    absLineSize = LINE_OUTSIDE_SIZE;
                }
            } else {
                if (smallestAngleBetween(lineDirection, absLineAngle) <= 90.0f) {
                    /* Same side of the line as last latch — refresh. */
                    absLineAngle = lineDirection;
                    absLineSize  = relLineSize;
                } else {
                    /* Opposite side — treat as exterior face of the line. */
                    absLineAngle = float_mod(lineDirection + 180.0f, 360.0f);
                    absLineSize  = 2.0f - relLineSize;
                }
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Strategy                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Attacker behaviour: orbit/chase ball, avoid line, face goal or hold heading.
 *
 * Movement is expressed in field frame then converted to robot frame for the mixer:
 *   motors.run(speed, float_mod(moveDir - bearing, 360), correction)
 *
 * Pipeline when ball is visible (@c ORBIT enabled):
 *   1. absBallDir = relBallDir + bearing (field frame)
 *   2. Orbit offset angle grows exponentially with angular error from
 *      orbitTarget (field forward or goal-biased offset)
 *   3. Distance multiplier shrinks orbit offset when ball is close (high strength)
 *   4. If ball is roughly ahead and close → surge straight at ball
 *   5. Else → moveDir = absBallDir + orbit angleAddition; speed scales with
 *      how much orbit is being applied
 *
 * Line handling (priority after ball calc):
 *   - No ball + deep on line → retreat opposite abs line (PID on size)
 *   - Ball + shallow line + move toward line → slide tangent (sin of angle)
 *   - Ball + deep line → forced retreat
 *   - Line present, other cases → retreat
 *
 * Rotation (moveCor):
 *   - Goal visible + GOAL_TRACKING → PID on attack goal argument
 *   - Else → IMU PID holding bearing ≈ 0 relative to @ref target
 */
void calculate_attack() {
    float moveDir = 0.0f;
    float moveSpd = 0.0f;
    float moveCor = 0.0f;

    float absBallDir = float_mod(relBallDir + bearing, 360.0f);

    if (relBallStr != 0.0f) {
        float orbitTarget = target;
        if (attackGoal.exists() && GOAL_TRACKING) {
            /* Bias orbit reference toward camera attack-goal bearing. */
            orbitTarget = float_mod(ORBIT_TARGET_OFFSET + bearing, 360.0f);
        }

        #if ORBIT
        /* Signed angular error robot→ball relative to desired orbit facing. */
        float dir = normaliseAngle180(float_mod(absBallDir - orbitTarget, 360.0f));

        /* Orbit angle addition: sign(dir) * min(90, multi * e^(exp*|dir|)). */
        float ballAngDiff =
            (dir > 0.0f ? 1.0f : -1.0f) * fmin(90.0f, ORBIT_DIR_MULTI * expf(ORBIT_DIR_EXP * fabs(dir)));

        /* strengthFactor → 1 when far/weak, → 0 when close/strong. */
        float strengthFactor = constrain(BALL_CLOSE_STR / fmax(relBallStr, 1.0f), 0.0f, 1.0f);
        float distMulti =
            constrain(ORBIT_DIST_MULTI * strengthFactor * expf(ORBIT_DIST_EXP * strengthFactor), 0.0f, 1.0f);
        float angleAddition = distMulti * ballAngDiff;

        if ((absBallDir < BALL_FRONT_MIN || absBallDir > BALL_FRONT_MAX) &&
            (relBallStr < BALL_STR_CLOSE_THRESH)) {
            /* Ball ahead and not yet "captured" — drive hard onto it. */
            moveDir = absBallDir;
            moveSpd = 255.0f;
            Serial.println("hi");
        } else {
            moveDir = float_mod(absBallDir + angleAddition, 360.0f);
        }

        /* More orbit offset → slower translation (keeps contact stable). */
        moveSpd = BASE_SPEED + (SURGE_SPEED - BASE_SPEED) * (1.0f - fabs(angleAddition / 90.0f));
        #endif
    }

    /* ---- Line avoidance / sliding (overrides translation as needed) ---- */
    if (absLineSize > LINE_AVOID_THRESH && relBallStr == 0.0f) {
        /* Lost ball while deep on white — escape field-inward. */
        moveDir = float_mod(absLineAngle + 180.0f, 360.0f);
        moveSpd = -lineAvoid.update(absLineSize, -1.0f);
    } else if (relBallStr != 0.0f) {
        if (absLineSize != -1.0f && absLineSize < LINE_AVOID_THRESH) {
            /* Shallow contact: if chasing toward the line, slide parallel instead. */
            if (smallestAngleBetween(moveDir, absLineAngle) < 90.0f) {
                moveSpd = sin(smallestAngleBetween(moveDir, absLineAngle) * DEG_TO_RAD) * LS_SLIDE_CONST;
                float difference = normaliseAngle180(float_mod(moveDir - absLineAngle, 360.0f));
                if (difference > 0.0f) {
                    moveDir = float_mod(absLineAngle + 90.0f, 360.0f);
                } else {
                    moveDir = float_mod(absLineAngle - 90.0f, 360.0f);
                }
            }
        } else if (absLineSize > LINE_AVOID_THRESH) {
            moveDir = float_mod(absLineAngle + 180.0f, 360.0f);
            moveSpd = -lineAvoid.update(absLineSize, -1.0f);
        }
    } else if (absLineSize != -1.0f) {
        moveDir = float_mod(absLineAngle + 180.0f, 360.0f);
        moveSpd = -lineAvoid.update(absLineSize, -1.0f);
    }

    /* ---- Heading / goal facing ---- */
    if ((attackGoal.exists() && GOAL_TRACKING)) {
        float goalAngle = normaliseAngle180(float_mod(attackGoal.arg, 360.0f));
        moveCor = goalTrack.update(goalAngle, 0.0f);
    } else {
        moveCor = -correction.update(normaliseAngle180(bearing), 0.0f);
    }

    #if DEBUG_MAIN_ATTACK
    Serial.printf("Move Dir: %.2f\tMove Spd: %.2f\tMove Cor: %.2f\n", moveDir, moveSpd, moveCor);
    #endif

    /* Field-frame direction → body-frame angle for DriveSystem mixer. */
    motors.run(moveSpd, float_mod(moveDir - bearing, 360.0f), moveCor);
    Serial.print(relBallDir);
    Serial.print("\t");
    Serial.print(relBallStr);
    Serial.print("\t");
    Serial.print(moveDir);
    Serial.print("\t");
    Serial.println(moveSpd);
}

/**
 * @brief Defender behaviour: stay between ball and own goal using orthogonal PIDs.
 *
 * Constructs a 2D velocity in a goal-aligned frame:
 *   - Horizontal: track ball azimuth (or hold heading if ball unseen)
 *   - Vertical:   hold preferred depth via abs line size, else camera goal range
 *
 * Speed = hypot(hozt, vert); direction = atan2(hozt, vert).
 * Correction faces away from defend goal (goal.arg + 180) when visible,
 * otherwise IMU hold.
 */
void calculate_defend() {
    float hoztInput = (relBallStr != 0.0f) ? -normaliseAngle180(relBallDir)
                                           :  normaliseAngle180(bearing);
    float hozt = horizontal.update(hoztInput, 0.0f);

    float vert = 0.0f;
    if (absLineSize != -1.0f) {
        vert = -vertical.update(absLineSize, 1.0f);
    } else if (defendGoal.exists()) {
        vert = vertCam.update(defendGoal.mag, DEFEND_CAM_TARGET);
    }

    float moveSpd = sqrtf(hozt * hozt + vert * vert);
    float moveDir = (atan2f(hozt, vert) * RAD_TO_DEG);
    float moveCor = 0.0f;

    if (defendGoal.exists()) {
        float goalAngle = float_mod(defendGoal.arg + 180.0f, 360.0f);
        moveCor = goalTrack.update(normaliseAngle180(goalAngle), 0.0f);
    } else {
        moveCor = -correction.update(normaliseAngle180(bearing), 0.0);
    }

    #if DEBUG_MAIN_DEFEND
    Serial.printf("Move Dir: %.2f\tMove Spd: %.2f\tMove Cor: %.2f\n", moveDir, moveSpd, moveCor);
    Serial.printf("Hozt: %.2f\tVert: %.2f\n", hozt, vert);
    #endif

    motors.run(moveSpd, float_mod(moveDir - bearing, 360.0f), moveCor);
}

/**
 * @brief Drive the battery warning LED from measured pack voltage.
 *
 * Above ROBOT_REQUIRED_VOLT: refresh the timer and force LED off.
 * Below threshold: after the timer elapses without a recovery, assert LED.
 * This avoids flicker from brief ADC dips while still flagging a true low pack.
 */
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

/* -------------------------------------------------------------------------- */
/* Arduino entry points                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief One-shot hardware bring-up.
 *
 * Order matters: IMU must leave reset / crystal settle before reliable heading.
 * Serial1 is the Secondary TSSP link; camera/motors/light/battery follow.
 * Digital pins for enable path, photogate, and battery LED are configured last.
 */
void setup() {
    state = STATE_IDLE;

    delay(100);
    Serial.begin(SERIAL_BAUD_RATE);

    while (!bno.begin(OPERATION_MODE_IMUPLUS)) {
        Serial.println("No BNO055 detected.");
        delay(1000);
    }
    delay(500);
    bno.setExtCrystalUse(true); /* higher heading stability vs internal osc */
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

/**
 * @brief Main control cycle — battery LED, enable gate, then FSM.
 *
 * motorsOn is derived either from the RCJ COM module ADC threshold or a
 * physical enable switch (USE_COM_MODULE). When disabled, motors are coast/brake
 * commanded to zero without leaving STATE_GAME (sensors keep updating).
 */
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
            /* --- IMU: yaw relative to calibrated field target --- */
            bno.getEvent(&event);
            bearing = float_mod(event.orientation.x - target, 360.0f);

            /* --- Vision --- */
            cam.update();
            attackGoal = cam.get_attack();
            defendGoal = cam.get_defend();

            #if not OPEN
            update_ball(); /* Secondary IR array */
            #else
            ballData   = cam.get_ball();
            relBallDir = ballData.arg;
            relBallStr = ballData.mag;
            #endif

            /* --- Line --- */
            ls.update();
            update_absolute_line();

            /* Role select placeholder — currently forced attacker. */
            bool attack = true;

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

            break;
        }
    };
}
