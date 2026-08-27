#include <Arduino.h>

void setup() {
  // pinMode(5, OUTPUT);
  // pinMode(6, OUTPUT);
  // pinMode(7, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
}

void loop() {
  // analogWrite(5, 100);
  // digitalWrite(7, HIGH);
  // digitalWrite(6, LOW);
  // Serial.println(digitalRead(24));
  digitalWrite(30, HIGH);
  digitalWrite(31, HIGH);
}