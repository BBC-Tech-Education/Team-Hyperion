#include <Arduino.h>
#include <Pins.h>
#include <Ball_handling.h>
// #include <Drive_system.h>

BallHandling ballHandler;



void setup() {
  ballHandler.init();
  pinMode(ENABLE_SWITCH, INPUT);
}

void loop() {
  // ballHandler.update();
  // Serial.println(digitalRead(ENABLE_SWITCH));
  if(digitalRead(ENABLE_SWITCH)) {
    ballHandler.run_dribbler(100.0f);
  } else {
    ballHandler.run_dribbler(0.0f);
  }
}

// ok enable switch now ok

// im literally writing power to them regardless of anything, so there has tobe something wrong wiring wise

// This has photogate as well btw - so it wont kick if the photogate is not activated
// speak herew
// is bluetooth 

// that doesnt do anything im literally just writing power to dribb at 100/255

// what does that do its writing to the pin manually- you cant print nothing

// no because this is a completely seperate file on its own

// click enable switch, but shouldnt kick until ball is in capture