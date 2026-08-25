#ifndef CONFIG_H
#define CONFIG_H

/////////////////////////////////// ROBOT ID ///////////////////////////////////
#define CONTROL 1 // control = 1, chaos = 0
#define GOAL_TRACKING 1
#define ORBIT 1 // enables or disables Orbit
#define SURGE 0 // enables or disables Surging
#define LOCALISATION 1
#define CAM_BLUE_GOAL 0
#define USE_COM_MODULE 0 // 1 = COM_MODULE pin, 0 = ENABLE_SWITCH

//////////////////////////////////// DEBUG ////////////////////////////////////

// --- Light System ---
#define DEBUG_LS_RAW 0
#define DEBUG_LS_THRESH 0
#define DEBUG_LS_ONWHITE 0
#define DEBUG_LS_CLUSTER 0
#define DEBUG_LS (DEBUG_LS_RAW || DEBUG_LS_THRESH || DEBUG_LS_ONWHITE || DEBUG_LS_CLUSTER)

// --- TSSP / Ball (Secondary) ---
#define DEBUG_TSSP_RAW 0
#define DEBUG_TSSP_PROCESS 0
#define DEBUG_TSSP_WEIGHTED 0
#define DEBUG_TSSP_BALL 0
#define DEBUG_TSSP (DEBUG_TSSP_RAW || DEBUG_TSSP_PROCESS || \
                    DEBUG_TSSP_WEIGHTED || DEBUG_TSSP_BALL)

// --- Drive ---
#define DEBUG_DRIVE_CMD 0
#define DEBUG_DRIVE_MOTORS 0
#define DEBUG_DRIVE (DEBUG_DRIVE_CMD || DEBUG_DRIVE_MOTORS)

// --- Camera ---
#define DEBUG_CAM_RAW 0
#define DEBUG_CAM (DEBUG_CAM_RAW)

// --- Bluetooth ---
#define DEBUG_BT_RX 0
#define DEBUG_BT_TX 0
#define DEBUG_BT_ROLE 0
#define DEBUG_BT (DEBUG_BT_RX || DEBUG_BT_TX || DEBUG_BT_ROLE)

// --- Primary main loop ---
#define DEBUG_MAIN_STATE 0
#define DEBUG_MAIN_IMU 0
#define DEBUG_MAIN_LINE 0
#define DEBUG_MAIN_GOALS 0
#define DEBUG_MAIN_ATTACK 0
#define DEBUG_MAIN_DEFEND 0
#define DEBUG_MAIN (DEBUG_MAIN_STATE || DEBUG_MAIN_IMU || DEBUG_MAIN_LINE || \
                    DEBUG_MAIN_GOALS || DEBUG_MAIN_ATTACK || DEBUG_MAIN_DEFEND)

#define DEBUG (DEBUG_LS || DEBUG_TSSP || DEBUG_DRIVE || DEBUG_CAM || DEBUG_BT || DEBUG_MAIN)


//////////////////////////////////// TUNING ////////////////////////////////////

#if CONTROL
//CONTROL
    #define BASE_SPEED 80.0f //110.0f
    #define SURGE_SPEED 120.0f //130.0f
    #define BALL_STR_CLOSE_THRESH 50.0f //20.0f
    #define BALL_CLOSE_STR 78.0f // higher = wider, lower = closer
    #define BALL_FRONT_MIN 15.0f //35.0f
    #define BALL_FRONT_MAX 350.0f //345.0f
    #define ORBIT_TARGET_OFFSET 45.0f
    #define ORBIT_DIR_MULTI 0.4f
    #define ORBIT_DIR_EXP 0.15f //0.25f
    #define ORBIT_DIST_MULTI 0.0125f // 0.0125f
    #define ORBIT_DIST_EXP 3.8f //4.5f

    #define SWITCHING_STRENGTH 20.0f
    #define DEFEND_CAM_TARGET 70.0
    #define DEFEND_MAX_DIST 465.07
    #define LS_THRESH 275
    #define KP_IMU 1.25
    #define KD_IMU 0.04 // 0.04
    #define KP_GOALT 1.0
    #define KD_GOALT 0.02
    #define KP_VERT 2.5
    #define KP_HOZT 1.8
    #define KP_CVERT 13.0
    #define KP_LOC 0.15
    #define KD_LOC 0.0
    #define KP_LAV 100.0
    #define KD_LAV 0.0
    #define COM_MODULE_THRESH 900
    #define LS_SLIDE_CONST 255.0f
    #define CAM_CENTER_X 244
    #define CAM_CENTER_Y 244
#else
// CHAOS
    #define BASE_SPEED 90.0f //110.0f
    #define SURGE_SPEED 110.0f //130.0f
    #define BALL_STR_CLOSE_THRESH 20.0f //20.0f
    #define BALL_CLOSE_STR 75.0f //65.0f
    // #define BALL_CLOSE_STR 30.0f // lower = narrower, higher = wider
    #define BALL_FRONT_MIN 35.0f //35.0f
    #define BALL_FRONT_MAX 330.f //345.0f
    #define ORBIT_TARGET_OFFSET 45.0f
    #define ORBIT_DIR_MULTI 1.20636f //0.4f
    #define ORBIT_DIR_EXP 0.0660224f //0.25f
    #define ORBIT_DIST_MULTI 12.41625f
    #define ORBIT_DIST_EXP -14.56002f

    #define DEFEND_CAM_TARGET 300.0
    #define DEFEND_MAX_DIST 465.07
    #define LS_THRESH 275
    #define KP_IMU 1.25
    #define KD_IMU 0.04 // 0.04
    #define KP_GOALT 0.8
    #define KD_GOALT 0.04
    #define KP_VERT 160.0
    #define KP_HOZT 1.2
    #define KP_CVERT 0.6
    #define KP_LOC 3.0f
    #define KD_LOC 0.0
    #define KP_LAV 100.0
    #define KD_LAV 0.0
    #define COM_MODULE_THRESH 900
    #define LS_SLIDE_CONST 255.0f
    #define CAM_CENTER_X 244
    #define CAM_CENTER_Y 244
#endif

// --- MAGIC NUMBERS & BEHAVIOR CONSTANTS ---
#define SERIAL_BAUD_RATE 9600
#define TSSP_BAUD_RATE 115200

#define BNO055_SENSOR_ID 55
#define IMU_PID_MAX 100.0f

#define GOALT_PID_MAX 100.0f
#define LAV_PID_MAX 150.0f
#define LOC_PID_MAX 80.0f

#define BATTERY_TIMER_INTERVAL 5000000

#define BALL_DIR_DIVISOR 100.0f

#define LINE_OUTSIDE_SIZE 3.0f
#define LINE_INSIDE_THRESH 1.0f
#define LINE_TOUCH_REENTRY_ANGLE 60.0f
#define LINE_AVOID_THRESH 0.2f

///////////////////////////////////// TSSP /////////////////////////////////////

#define TSSP_NUM 24
#define BALL_DUTY_CYCLE 0.35f
#define READ_PERIOD 20
#define TSSP_OFFSET 15
#define PROCESS_PERIOD 10000
#define ALPHA 0.4f
#define TSSP_STR_FLOOR 20.0f

#define TSSP_PACKET_SIZE 5
#define TSSP_START_BYTE_1 255
#define TSSP_START_BYTE_2 255


//////////////////////////////////// MOTORS ////////////////////////////////////

#define MOTOR_NUM 4
#define MOTOR_ANALOG_FRQ 15000.0f


///////////////////////////////// LIGHT SENSORS ////////////////////////////////

#define LS_NUM 48
#define LS_INNER_NUM 32


//////////////////////////////////// CAMERA ////////////////////////////////////

#define CAM_SERIAL Serial5
#define CAM_PACKET_SIZE 8
#define CAM_START_BYTE_1  255
#define CAM_START_BYTE_2 250
#define CAM_PIXEL_SHIFT 120

////////////////////////////////// BLUETOOTH ///////////////////////////////////

// Serial2 on Primary Serial1 is inter-Teensy ball link, Serial5 is camera.
#define BT_SERIAL Serial2
#define BT_BAUD 115200
#define BT_PACKET_SIZE 6
#define BT_START_BYTE 255
#define BT_FLAG_ROLE 0x01
#define BT_CONNECTION_TIMEOUT_US 1000000UL
#define BT_SEND_TIMER_US 10000
#define BT_ROLE_CONFLICT_TIMER_US 1000000UL


/////////////////////////////// VOLTAGE DIVIDERS ///////////////////////////////

#define ROBOT_VOLTAGE_STABALISER 73.41388
#define ROBOT_VOLTAGE_OFFSET 0.215224
#define ROBOT_REQUIRED_VOLT 11.2

/////////////////////////////// FIELD DIMENSIONS ///////////////////////////////
#define FIELD_LENGTH_MM 2400


#endif