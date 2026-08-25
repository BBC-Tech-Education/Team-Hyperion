#include <Tssp.h> //tssp header file
#include <Light_Sensor.h> //light sensor header file
#include <Motor.h> //Motor header file
#include <Camera.h> //Camera header file

Tssp tssp_1 = Tssp(13); //creating tssp object
Light_Sensor ls_1 = Light_Sensor(12); //creating light sensor object
Motor motor1 = Motor(11, 10, 9); //creating motor object

void setup() {
  Serial.begin(9600);
  tssp_1.init(); //initializing tssp
  ls_1.init(); //initializing light sensor
  motor1.init(); //initializing motor
}

void loop() {
  int ir_value = tssp_1.read(); //reading tssp consistantly
  int ls_value = ls_1.read(); //reading light sensor consistantly
  motor1.run(155, HIGH, LOW); //run motor clockwise
  Serial.println(1);
}