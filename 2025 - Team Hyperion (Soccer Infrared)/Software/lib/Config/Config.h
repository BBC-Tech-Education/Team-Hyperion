/*!
 * @file config.h
 * 
 * @mainpage This is a library that includes variables and defines that change 
 * the code's behaviour.
 * 
 * @date 09/05/25
 * 
 * @author S.Garg (Brisbane Boys' College)
 * @author T.McCabe (Brisbane Boys' College)
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Common.h>

// --[MASTER Values]--
#define SECOND_ROBOT true
#define GOAL_TRACKING_TOGGLE true
#define DEBUG_CORE_CODE true

#if not SECOND_ROBOT
#define SURGE_SPEED 100
#define BASE_SPEED 90
#else
#define SURGE_SPEED 100
#define BASE_SPEED 90
#endif

// --[STRATEGIC Values]--
#define ATTACKING true
#define NEUTRAL_POINT_MOVE false

// --[DEBUG TOGGLES]--
#define DEBUG_BLUETOOTH false
#define DEBUG_CAMERA false
#define DEBUG_MOTORS false
#define DEBUG_LS_VALS false
#define DEBUG_LS_TRIG false
#define DEBUG_LS_CALCS false
#define DEBUG_TSSP_VALS  false
#define DEBUG_VD false

// --[PID Values]--
#if not SECOND_ROBOT
#define KP_IMU 1.0
#define KD_IMU 0.08
#define KP_CAM_ATTACK 1.0
#define KD_CAM_ATTACK 0.1
#define KP_CAM_DEFEND 1.0
#define KD_CAM_DEFEND 0.1
#define SP_DEFEND_VERT 26.0
#define KP_DEFEND_VERT 9.0
#define KD_DEFEND_VERT 0.0
#define KP_DEFEND_HOZT 1.5
#define KD_DEFEND_HOZT 0.0
#define ABSM_DEFEND_HOZT 100.0
#define ATK_LINE_SP 0.3
#define KP_LINE_AVOID 150.0
#define SP_VERT_CENTERING 43.0
#define KP_VERT_CENTERING 15.0
#define KP_HOZT_CENTERING 6.0
#else
#define KP_IMU 1.0
#define KD_IMU 0.08
#define KP_CAM_ATTACK 1.0
#define KD_CAM_ATTACK 0.1
#define KP_CAM_DEFEND 1.0
#define KD_CAM_DEFEND 0.1
#define SP_DEFEND_VERT 31.0
#define KP_DEFEND_VERT 9.0
#define KD_DEFEND_VERT 0.0
#define KP_DEFEND_HOZT 1.5
#define KD_DEFEND_HOZT 0.0
#define ABSM_DEFEND_HOZT 100.0
#define ATK_LINE_SP 0.3
#define KP_LINE_AVOID 200.0
#define SP_VERT_CENTERING 37.0
#define KP_VERT_CENTERING 15.0
#define KP_HOZT_CENTERING 6.0
#endif

// --[LOGIC Values]
#define SWITCHING_STRENGTH 140.0
#if not SECOND_ROBOT
#define ORBIT_STRENGTH_RADIUS 135.0
#define DEF_START_SURGE 140.0
#define DEF_KEEP_SURGE_UNTIL 100.0
#else
#define ORBIT_STRENGTH_RADIUS 125.0
#define DEF_START_SURGE 140.0
#define DEF_KEEP_SURGE_UNTIL 100.0
#endif

// --[LIGHT SENSOR Values]--=
#if not SECOND_ROBOT
#define LS_CLB_THRESH 300
#define LS_FLIP_THRESH 90
#else
#define LS_CLB_THRESH 300
#define LS_FLIP_THRESH 90
#endif

// --[BLUETOOTH Values]
#define BT_SERIAL Serial1
#define BT_BAUD 9600
#define BT_PACKET_SIZE 6
#define BT_START_BYTE 255
#define BT_NO_DATA 255

// --[CAMERA Values]--
#define cameraSerial Serial8
#define CAM_PACKET_SIZE 6
#define CAM_START_PACK_1 200
#define CAM_START_PACK_2 122

// --[BATTERY TRACKING Values]--
#define BATTERY_CRITICAL 11.0
#define BATTERY1_DIVIDER 71

#endif
