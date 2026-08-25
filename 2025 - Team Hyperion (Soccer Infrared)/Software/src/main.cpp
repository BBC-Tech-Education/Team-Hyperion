#include <Drive_system.h>
#include <Light_system.h>
#include <Adafruit_BNO055.h>
#include <VoltDiv.h>
#include <Timer.h>
#include <Bluetooth.h>
#include <Camera.h>
#include <TSSP_system.h>
#include <Debug.h>


Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_B, &Wire);
Bluetooth bt;
Camera cam;
Drive_system motors;
Light_system ls;
PID avoidLine(KP_LINE_AVOID, 0.0, 0.0, 200.0);
PID defendHozt(KP_DEFEND_HOZT, 0.0, KD_DEFEND_HOZT, ABSM_DEFEND_HOZT);
PID defendVert(KP_DEFEND_VERT, 0.0, KD_DEFEND_VERT);
PID attackCor(KP_CAM_ATTACK, 0.0, KD_CAM_ATTACK);
PID defendCor(KP_CAM_DEFEND, 0.0, KD_CAM_DEFEND);
PID bearingCor(KP_IMU, 0.0, KD_IMU, 100.0);
PID centeringPIDHozt(KP_HOZT_CENTERING, 0.0, 0.0, 80.0);
PID centeringPIDVert(KP_VERT_CENTERING, 0.0, 0.0);
Timer batteryTimer(5000000);    
Tssp_system tssp;
VoltDiv battery(BATT_READ_PIN, BATTERY1_DIVIDER);
sensors_event_t bearing;
Debug debug;

bool isSurging = false;

void setup() {
    debug.init(9600);
    if(DEBUG_CORE_CODE) {
        // Type any of the below strings into the terminal (when monitoring) and
        // the program sends out the corresponding value
        debug.addMode("ballDir", []() { return String(tssp.getBallDir()); });
        debug.addMode("ballStr", []() { return String(tssp.getBallStr()); });
        debug.addMode("compass", []() { return String(bearing.orientation.x); });
        debug.addMode("switch", []() { return String(bt.getRole()); });
        debug.addMode("camAtkAng", []() { return String(cam.getAttackGoalAngle()); });
        debug.addMode("camAtkDis", []() { return String(cam.getAttackGoalDist()); });
        debug.addMode("camDefAng", []() { return String(cam.getDefendGoalAngle()); });
        debug.addMode("camDefDis", []() { return String(cam.getDefendGoalDist()); });
        debug.addMode("lsDir", []() { return String(ls.getLineDirection()); });
        debug.addMode("lsState", []() { return String(ls.getLineState()); });
        debug.addMode("batLvl", []() { return String(battery.getLvl()); });

        // After you are done monitoring that specific variable and want to do
        // another, you can simply say "stop" in the terminal, and it stops.
        
        // If you want to see multiple things at the same time, you can seperate
        // them by commas, for example:
        // "switch,camAtkDis,camDefDis" --> outputs the corresponding functions
    }

    tssp.init();
    ls.init();
    motors.init();
    bt.init();
    battery.init();
    batteryTimer.update();
    cam.init();
    pinMode(GOAL_PIN, INPUT);
    while(!bno.begin(OPERATION_MODE_IMUPLUS)) {
        Serial.println("No BNO055 detected. Check your wiring or I2C ADDR.");
        delay(1000);
    }
    delay(500);
    bno.setExtCrystalUse(true);
    delay(500);
}


void loop() {
    tssp.update();
    bt.update(tssp.getBallDir(), tssp.getBallStr(), true);
    bno.getEvent(&bearing);
    // cam.update(digitalRead(GOAL_PIN));
    cam.update(false);
    // Serial.println((digitalRead(GOAL_PIN)));
    ls.update(bearing.orientation.x, true);
    battery.update();
    float moveDir = 0.0;
    float moveSpeed = 0.0;
    float correction = -bearingCor.update((bearing.orientation.x > 180) ? bearing.orientation.x - 360  : bearing.orientation.x, 0.0);

    if (!isSurging && tssp.getBallStr() >= DEF_START_SURGE) {
        isSurging = true;   // Start surging
    } 
    else if (isSurging && tssp.getBallStr() < DEF_KEEP_SURGE_UNTIL) {
        isSurging = false;  // Stop surging
    }
    if(bt.getRole() || ATTACKING) {
    // if(false) {
        if((tssp.getBallStr() == 0) && GOAL_TRACKING_TOGGLE && NEUTRAL_POINT_MOVE) {
            float hoztVect = -centeringPIDHozt.update(cam.getAttackGoalAngle() > 180.0 ? cam.getAttackGoalAngle() - 360.0 :
                cam.getAttackGoalAngle(), 0.0);
            float vertVect = centeringPIDVert.update(SP_VERT_CENTERING - cam.getAttackGoalDist(), 0.0);
            moveDir = cam.getAttackGoalVisible()?floatMod(atan2f(hoztVect, vertVect) * RAD_TO_DEG, 360.0):0;
            moveSpeed = cam.getAttackGoalVisible()?sqrtf(powf(vertVect, 2) + powf(hoztVect, 452)):SURGE_SPEED/2;
        } else {
            if(cam.getAttackGoalVisible() && GOAL_TRACKING_TOGGLE && !(tssp.getBallDir() > 40 && tssp.getBallDir() < 320)) {
                correction = attackCor.update(cam.getAttackGoalAngle() > 180.0 ? cam.getAttackGoalAngle() - 360.0 :
                    cam.getAttackGoalAngle(), 0.0);
            }
            tssp.orbit();
            moveDir = tssp.getMoveDir();
            moveSpeed = tssp.getMoveSpd();
        }
    } else {
        if (isSurging && (tssp.getBallDir() < 30 || tssp.getBallDir() > 330)
            && (cam.getDefendGoalDist() <= (SP_DEFEND_VERT + 8.0))) {
            moveDir = tssp.getBallDir();
            moveSpeed = 200.0;
            if(cam.getAttackGoalVisible() && GOAL_TRACKING_TOGGLE && !(tssp.getBallDir() > 40 && tssp.getBallDir() < 320)) {
                correction = attackCor.update(cam.getAttackGoalAngle() > 180.0 ? cam.getAttackGoalAngle() - 360.0 :
                    cam.getAttackGoalAngle(), 0.0);
            }
        } else if((tssp.getBallDir() < 310 && tssp.getBallDir() > 130)) {
            tssp.orbit();
            moveDir = tssp.getMoveDir();
            moveSpeed = tssp.getMoveSpd();
        } else {
            float vertVect = defendVert.update(cam.getDefendGoalDist() - SP_DEFEND_VERT, 0);
            float defHeading = (tssp.getBallStr() != 0) ? ((tssp.getBallDir() > 180) ? tssp.getBallDir() - 360 : tssp.getBallDir()) : -((bearing.orientation.x > 180) ? bearing.orientation.x - 360 : bearing.orientation.x);
            float hoztVect = -defendHozt.update(defHeading, 0.0);
            if(cam.getDefendGoalVisible()) {
                moveDir = floatMod(atan2f(hoztVect, vertVect)*RAD_TO_DEG, 360);
                moveSpeed = sqrtf(powf(vertVect, 2) + powf(hoztVect, 2));
            } else {
                moveDir = 180.0;
                moveSpeed = 50.0;
            }
            if(cam.getDefendGoalVisible() && GOAL_TRACKING_TOGGLE) {
                float target = floatMod(cam.getDefendGoalAngle() + 180.0, 360.0);
                correction = defendCor.update(target > 180 ? target - 360.0 : 
                                    target, 0.0);
            }
        }
    }

    if(ls.getLineState() > (SECOND_ROBOT?0.0:0.75)) {
        moveDir = floatMod(ls.getLineDirection() + 180.0 - bearing.orientation.x, 360.0);
        moveSpeed = 200.0;
    } else if (ls.getLineState() > 0.0) {
        if(smallestAngleBetween(floatMod(moveDir + bearing.orientation.x, 360.0), ls.getLineDirection()) < 60) {
            moveDir = floatMod(ls.getLineDirection() + 180.0 - bearing.orientation.x, 360.0);
            moveSpeed = abs(avoidLine.update(ls.getLineState(), ATK_LINE_SP));
        }
    }

    if(DEBUG_CORE_CODE) {
        debug.update();
    }

    if (battery.getLvl() <= BATTERY_CRITICAL && batteryTimer.timeHasPassedNoUpdate()) {
        moveSpeed = 0;
        moveDir = 0;
        correction = 20;
    } else {
        batteryTimer.update();
    }
    Serial.println(battery.getLvl());

    // Serial.println(tssp.getBallStr());
    motors.run(moveSpeed, moveDir, correction);
}