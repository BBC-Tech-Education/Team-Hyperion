#include <Arduino.h>
#include <Pins.h>
#include <Common.h>
#include <Drive_system.h>

DriveSystem motors;

float dir = 0.0f;

void setup() {
  motors.init();
}

void loop() {
  dir = float_mod(dir, 360.0f);
  motors.run(50.0f, dir, 0.0f);
  delay(10);
  dir++;
}