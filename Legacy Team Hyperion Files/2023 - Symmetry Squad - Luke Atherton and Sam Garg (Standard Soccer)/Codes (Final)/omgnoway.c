#pragma config(Sensor, S1,     frontIR,        sensorI2CCustom)
#pragma config(Sensor, S2,     backIR,         sensorI2CCustom)
#pragma config(Sensor, S3,     Compass,        sensorEV3_GenericI2C)
#pragma config(Sensor, S4,     UltraS,         sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          BRmotor,       tmotorEV3_Large, openLoop, reversed, encoder)
#pragma config(Motor,  motorB,          FRmotor,       tmotorEV3_Large, openLoop, reversed, encoder)
#pragma config(Motor,  motorC,          FLmotor,       tmotorEV3_Large, openLoop, reversed, encoder)
#pragma config(Motor,  motorD,          BLmotor,       tmotorEV3_Large, openLoop, reversed, encoder)
#include "hitechnic-irseeker-v2.h"
#include "PF_Motors.h"
#include "PF_Motors.c"
#define COMP_MULTI 0.02
int speed = 100;
int set = 20;
int motorSpeeds[18][4] = { //Defining movement
	{ -speed, -speed,  speed,  speed}, // Forward 0
	{ -speed,   set, -speed,   set}, // Forward-Right 1
	{ -speed,  speed,  speed, -speed}, // Right 2
	{   set,  speed,   set, -speed}, // Backward - Right 3
	{  speed,  speed, -speed, -speed}, // Backward 4
	{  speed,   set, -speed,   set}, // Backward - Left 5
	{  speed, -speed, -speed,  speed}, // Left 6
	{   set, -speed,   set,  speed}, // Forward-Left 7
	{    0,    0,    0,    0}, // Still 8
	{  speed,  speed, -speed, -speed}, // Forward 9
	{  speed,    0, -speed,    0}, // Forward-Right 10
	{  speed, -speed, -speed,  speed}, // Right 11
	{    0, -speed,    0,  speed}, // Backward-Right 12
	{ -speed, -speed,  speed,  speed}, // Backward 13
	{ -speed,    0,  speed,    0}, // Backward-Left 14
	{ -speed,  speed,  speed, -speed}, // Left 15
	{    0,  speed,    0, -speed}, // Forward-Left 16
	{    0,    0,    0,    0}, // Still 1
};
int motorVal[4] = {0, 0, 0, 0};
float max(float a, float b) {
	return a > b? a : b;
}
float uDis;
float mDis;
float caluculateCorrection(int target, int current) { //Coverting values for compass correct
	float correction = (target - current) * 2;
	if(correction > 180) {
		correction -= 360;
	}
	else if(correction < -180) {
		correction += 360;
	}
	correction *= COMP_MULTI;
	correction *= -1;
	correction += 22.25*(uDis*(cos((current*3.14)/180)-mDis));
	return correction;
}
int calculateBallDir(int frontStr, int backStr, int frontDir, int backDir) { //Change 1 & 10 if IR messes up
	int ballDir;
	if(frontStr >= backStr){
		switch(frontDir) {
		case 1:
			ballDir = 6;
			break;
		case 2:
			ballDir = 5; //Backward Left // Forward Right
			break;
		case 3:
			ballDir = 6; //Left // Right
			break;
		case 4:
			ballDir = 7; //Forward Left // Forward Right
			break;
		case 5:
			ballDir = 0; //Forward // Forward
			break;
		case 6:
			ballDir = 1; //Forward Right // Forward Left
			break;
		case 7:
			ballDir = 2; //Right // Left
			break;
		case 8:
			ballDir = 3; //Backward Right // Forward Left
			break;
		case 9:
			ballDir = 6; //Left // Left
			break;
		default:
			ballDir = 4;  //Back // Back
		}
	}
	else
	{
		switch(backDir) {
		case 1:
			ballDir = 16;//Forward Left // Backward Left
			break;
		case 2:
			ballDir = 17;//
			break;
		case 3:
			ballDir = 9;
			break;
		case 4:
			ballDir = 10;
			break;
		case 5:
			ballDir = 11;
			break;
		case 6:
			ballDir = 12;
			break;
		case 7:
			ballDir = 9;
			break;
		case 8:
			ballDir = 10;
			break;
		case 9:
			ballDir = 11;
			break;
		default:
			ballDir = 4;
		}
	}
	return ballDir;
}
void moving(int ballDir, float correction) { //Hard code
	for(int i = 0; i < 4; i++) {
		motorVal[i] = motorSpeeds[ballDir][i];
		motorVal[i] -= correction;
	}
	float highestVal = max(abs(motorVal[0]), abs(motorVal[1]));
	highestVal = max(highestVal, abs(motorVal[2]));
	highestVal = max(highestVal, abs(motorVal[3]));
	if(highestVal > 100) {
		if(highestVal != 0) {
			for(int i = 0; i <= 3; i++) {
				motorVal[i] /= highestVal;
				motorVal[i] *= 100;
			}
		}
	}
	motorIn.structure.A = motorVal[0]; //Convert
	motorIn.structure.B = motorVal[1];
	motorIn.structure.C = motorVal[2];
	motorIn.structure.D = motorVal[3];
}

task main()
{
	tHTIRS2 irSeeker1;
	tHTIRS2 irSeeker2;
	initSensor(&irSeeker1, frontIR);
	initSensor(&irSeeker2, backIR);
	mDis = 85.7;
	float correction;
	float uDis;
	int ballDir;
	int target = SensorValue[S3];
	int current = target;

	for(int i = 0; i < 4; i++) {
		motorIn.array[i] = 0;
	}
	initPFMotors();
	startMotorTask();
	bool ignoreLoop = false;
	while (true) {
		writeDebugStream("cor,ta,cu %4d %4d %4d\n", ballDir);
		readSensor(&irSeeker1);
		readSensor(&irSeeker2);
		current = SensorValue[S3];
		uDis = SensorValue[S4];
		ballDir = calculateBallDir(irSeeker1.enhStrength, irSeeker2.enhStrength, irSeeker1.acDirection, irSeeker2.acDirection);
		correction = caluculateCorrection(target, current);
		moving(ballDir, correction);
		if (uDis == 255 || uDis < 5) {
			ignoreLoop = true;
			motorIn.structure.A = set;
			motorIn.structure.B = 100;
			motorIn.structure.C = 30;
			motorIn.structure.D = -100;
			wait1Msec(300);
			ignoreLoop = false;
		}
	}
	// Code continues here after the loop
	if (!ignoreLoop) {
		// Add any additional code you want to execute after the loop
	}
}
