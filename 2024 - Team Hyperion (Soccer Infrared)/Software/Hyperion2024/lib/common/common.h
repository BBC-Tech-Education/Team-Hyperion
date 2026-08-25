#include <math.h>
#include <Arduino.h>

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352
#define CIRCLE_DEGREES 360
#define SEMI_CIRCLE_DEGREES 180

#define PID_p 1 //how fast it goes towards the point that you want - tune this
#define PID_i 0
#define PID_d 0.05 //how fast it slows down once it gets close to the 'p' point - tune this
#define PID_abs_max 100 //The cap value that the PID can produce from the numbers given

#define ANALOG_DIV_100 2.55
#define SPEED_GRADIENT 100/-255
#define NEGATIVE_SPEED_GRADIENT -1
#define SPEED_Y_INTERCEPT 68
#define ADD_OR_SUBTRACT_ORBIT 60

#define green_higherthan_thresh 0
#define green_lowerthan_thresh 0

float floatMod(float x, float m);
 
float angleBetween(float angleCounterClockwise, float angleClockwise);
 
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise);
 
int8_t findSign(float value);
 
float midAngleBetween(float angleCounterClockwise, float angleClockwise);
 
float smallestAngleBetween(float angleCounterClockwise, float angleClockwise);
 
int8_t findSign(float value);
 
bool angleIsInside(float angleBoundCounterClockwise, float angleBoundClockwise, float angleCheck);

#define MPU_CALIBRATION_COUNT 20
#define MPU_CALIBRATION_TIME 50
#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C
#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

#define NUM_LS 16
#define NUMBER_MUX 4
#define LS_OFFSET 180
#define NUM_TSSP 12

#define MOTOR_IGNORE_MOVEMENT 95
#define MOTOR_FLAG_COUNTER 1
#define SET_MOVE_DIR 0
#define MOVEMENTSPEED 100

#define FRANGLE 45
#define BRANGLE 135
#define BLANGLE 225
#define FLANGLE 315

#define MOTOR_NUM 4


//struct movingAngle {
//    float moving_angle = 0;
//};