//Includes
#include <light_system.h> //Light system header file
#include <tssp_system.h> //tssp system header file
#include <drive_system.h> //motor system header file
#include <camera.h> //camera header file
#include <Wire.h> //BNO File Header
#include <Adafruit_Sensor.h> //BNO File Header
#include <Adafruit_BNO055.h> //BNO File Header
#include <imumaths.h> //BNO Math File Header
#include <PID.h> //PID master file
#include <orbit.h> //Orbit master file
#include <math.h> //Advanced Mathematics File Header (cos, sqrt, pi)
#include <pins.h> //Pins master file
#include <common.h> //Common master file

//Objects
Tssp_system tssp = Tssp_system(TSSP_PIN); //Creating Tssp System
Light_System ls = Light_System(); //Creating Light System
Drive_system motors = Drive_system(); //Creating Drive System
bno::Adafruit_BNO055 compass; //Creating BNO
sensors_event_t direction; //Creating BNO
PID compass_correct(PID_p, PID_i, PID_d, PID_abs_max);
Orbit orbit;
mainThings mainthings;