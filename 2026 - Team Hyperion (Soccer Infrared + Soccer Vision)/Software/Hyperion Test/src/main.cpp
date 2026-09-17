#include <Arduino.h>
#include <Pins.h>

void setup() {
  pinMode(PHOTOGATE_PIN, INPUT);
}

void loop() {
  Serial.println(analogRead(PHOTOGATE_PIN));
}