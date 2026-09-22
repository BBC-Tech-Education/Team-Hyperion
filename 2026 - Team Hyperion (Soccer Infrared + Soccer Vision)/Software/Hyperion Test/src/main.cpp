#include <Arduino.h>
#include <Pins.h>
#include <Ball_handling.h>

BallHandling ballHandler;

void setup() {
  ballHandler.init();
  pinMode(ENABLE_SWITCH, INPUT);
}

void loop() {
  Serial.print(analogRead(KICKER_VD_PIN));
  Serial.print("\t");
  // Serial.println(analogRead(PHOTOGATE_PIN));
  ballHandler.update();
  Serial.print(ballHandler.get_current_kicks());
  Serial.print("\t");
  Serial.println(ballHandler.can_kick());
  if(digitalRead(ENABLE_SWITCH)) {
    ballHandler.kick();
  }
}