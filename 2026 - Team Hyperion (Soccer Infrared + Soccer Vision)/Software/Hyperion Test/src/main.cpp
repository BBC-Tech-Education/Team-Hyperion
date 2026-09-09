#include <Arduino.h>
#include <Pins.h>
#include <Common.h>

#define QUICK_CHANGE 0

float modSeconds;

void setup() {
  // pinMode(5, OUTPUT);
  // pinMode(6, OUTPUT);
  pinMode(FLINA, OUTPUT);
  pinMode(FLPWM, OUTPUT);
  pinMode(FLINB, OUTPUT);
  pinMode(FRINA, OUTPUT);
  pinMode(FRPWM, OUTPUT);
  pinMode(FRINB, OUTPUT);
  pinMode(BLINA, OUTPUT);
  pinMode(BLPWM, OUTPUT);
  pinMode(BLINB, OUTPUT);
  pinMode(BRINA, OUTPUT);
  pinMode(BRPWM, OUTPUT);
  pinMode(BRINB, OUTPUT);
}

void loop() {
  // analogWrite(5, 100);
  // digitalWrite(7, HIGH);
  // digitalWrite(6, LOW);
  // Serial.println(digitalRead(24));
  float seconds = millis() / 1000.0f;
  modSeconds = float_mod(seconds, 3.0f);
  // Serial.print(modSeconds);
  // Serial.print("\t");
  uint8_t inAHigh = LOW;
  uint8_t inBHigh = LOW;
  #if QUICK_CHANGE
  if(modSeconds < 1.0f) {
    inAHigh = LOW;
    inBHigh = HIGH;
  } else if(modSeconds < 1.5f && modSeconds > 1.0f) {
    inAHigh = LOW;
    inBHigh = LOW;
  } else if(modSeconds > 1.5f && modSeconds < 2.5f) {
    inAHigh = HIGH;
    inBHigh = LOW;
  } else {
    inAHigh = LOW;
    inBHigh = LOW;
  }
  #else
  if(modSeconds < 1.0f) {
    inAHigh = LOW;
    inBHigh = HIGH;
  } else if(modSeconds < 1.5f && modSeconds > 1.0f) {
    inAHigh = LOW;
    inBHigh = LOW;
  } else if(modSeconds > 1.5f && modSeconds < 2.5f) {
    inAHigh = HIGH;
    inBHigh = LOW;
  } else {
    inAHigh = LOW;
    inBHigh = LOW;
  }
  #endif

  // uint8_t inAHigh = (modSeconds > 1.5f) ? HIGH : LOW;
  // Serial.print(inAHigh);
  // Serial.print("\t");
  // Serial.println(!inAHigh);

  analogWrite(FRPWM, 50);
  analogWrite(FLPWM, 50);

  digitalWrite(FRINA, inAHigh);
  digitalWrite(FLINA, inAHigh);
  digitalWrite(FRINB, inBHigh);
  digitalWrite(FLINB, inBHigh);

  analogWrite(BRPWM, 50);
  analogWrite(BLPWM, 50);
  
  digitalWrite(BRINA, inAHigh);
  digitalWrite(BLINA, inAHigh);
  digitalWrite(BRINB, inBHigh);
  digitalWrite(BLINB, inBHigh);
}