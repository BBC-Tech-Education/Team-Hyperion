#include <Arduino.h>
#include <Pins.h>
#include <Ball_handling.h>
#include <Drive_system.h>

BallHandling ballHandler;

DriveSystem motors;


void setup() {
  // ballHandler.init();
  // pinMode(ENABLE_SWITCH, INPUT);
  // motors.init();

  pinMode(FLINA, OUTPUT);
  pinMode(FLINB, OUTPUT);
  pinMode(FLPWM, OUTPUT);
}

void loop() {
  // Serial.print(analogRead(KICKER_VD_PIN));
  // Serial.print("\t");
  // ballHandler.update();
  // Serial.print(ballHandler.get_current_kicks());
  // Serial.print("\t");
  // Serial.print(ballHandler.can_kick());
  // Serial.print("\t");
  // if(digitalRead(ENABLE_SWITCH)) {
  //   ballHandler.kick();
  // }
  // Serial.print(analogRead(PHOTOGATE_PIN));
  // Serial.print("\t");
  // Serial.println(ballHandler.photogate_triggered());
  // Serial.println(analogRead(PHOTOGATE_PIN));

  // motors.run(100.0f, 0.0f, 0.0f);

  digitalWrite(FLINA, HIGH);
  digitalWrite(FLINB, LOW);
  analogWrite(FLPWM, 100);
  
}