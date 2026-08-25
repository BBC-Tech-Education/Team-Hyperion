#include <Light_Sensor.h> //light sensor header file

Light_Sensor ls_1 = Light_Sensor(12); //creating light sensor object

void setup() {
  ls_1.init();
  Serial.begin(9600);
}

void loop() {
  int ls_value = ls_1.read(); //reading light sensor consistantly
  Serial.println(ls_value);
}