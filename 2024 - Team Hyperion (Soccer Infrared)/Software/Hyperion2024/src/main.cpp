#include <light_system.h>
#include <tssp_system.h>
#include <drive_system.h>
#include <camera.h>
#include <PID.h>
#include <orbit.h>
#include <pins.h>
#include <common.h>
#include <MPU.h>
#include <initialize.h>

class Tssp_system tssp = Tssp_system();
Light_System ls = Light_System();
Drive_system motors = Drive_system();
PID compass_correct(PID_p, PID_i, PID_d, PID_abs_max);
Orbit orbit;
MPU compass = MPU();
initialize config = initialize();

float compassVal = 0; //constant value of MPU
float targetValue = 0; //target value of the MPU
float ballDirection = 0; //ball direction
float moveDir = 0; //move direction
float prevDir = 0; //prev direction
float ballSpeedNum = 0; //tssp strength
float moveSpeed = 0;//speed to move at
int ballStr = 0; //ball strength
int lineAvoidance = 0; //lineavoidance movement value
float lineAvoidance2 = 0; //lineavoidance movement value 2
double correction = 0; //how much to correct value

void initialize::calculate_final_movement() {
  //calculating the final movement based on other values
  // lineAvoidance2 = floatMod(prevDir+180, 360);
  // if (ls.lineAngle != -1) {
  //   motors.update(moveSpeed, moveDir, 0, correction);
  // } else {
  //   motors.update(moveSpeed, moveDir, 0, correction);
  //   prevDir = moveDir;
  // }
  motors.update(100, 0, 0, 0);
}
void initialize::reading_values() {
  //anything you want to read in the forever loop goes here
  compass.update();
  ballDirection = tssp.read();
  compassVal = compass.heading;
  targetValue = compass.targetVal;
  ballStr = tssp.tsspStrength;
  ls.read();
}

/*
void Light_System::read() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        for (int j = 0; j < NUM_MUX; j++) {
            digitalWrite(muxList[j], (i >> j) & 0x01);
        }
        delayMicroseconds(10);
        lightData[i] = analogRead(LIGHT_PIN);
        Serial.print(lightData[i]);
        Serial.print(" ");
    }
    Serial.println();
}
*/

void initialize::initialize_sensors() {
  //anything you want to intialize in the setup goes in here
  Serial.begin(9600);
  ls.init();
  tssp.init();
  motors.init();
  compass.init();
  compass.findTargetVal();
}
void initialize::printing() {
  //any thing you want to print goes in here
  // for (int i = 0; i < 16; i++) {
  //   Serial.print(ls.lightData[i]);
  //   Serial.print("\t");
  // }
  // Serial.println();
}
void initialize::logic() {
  //anything that you want to calculate with functions goes in here
  moveDir = orbit.calculate_Direction3(ballDirection);
  lineAvoidance = ls.update();
  moveSpeed = orbit.calculate_Speed(ballStr, ballDirection);
  correction = compass_correct.update(compass.heading > 180 ? compass.heading - 360 : compass.heading, 0) - 5;
}

        // if(i < 8){
        //     lightData[15 - i] = analogRead(LIGHT_PIN);
        // } else {
        //     lightData[i - 8] = analogRead(LIGHT_PIN);
        // }

void setup() {
  config.initialize_sensors();
}

void loop() {
  config.reading_values();
  config.logic();
  config.calculate_final_movement();
  config.printing();
}