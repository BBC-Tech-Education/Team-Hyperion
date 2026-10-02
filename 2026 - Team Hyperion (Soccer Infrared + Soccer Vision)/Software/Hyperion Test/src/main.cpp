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
  ballHandler.update(80.0f);
  Serial.print(ballHandler.kicker_ready());
  Serial.print("\t");
  Serial.println(ballHandler.can_kick());
  if(digitalRead(ENABLE_SWITCH)) {
    ballHandler.kick();
  }
}

// it will not kick without the enable switch being on, so if you want it to
// dribble and then kick, you can leave the enable switch off and then turn it on when you want to kick
// up to you how you want to do it,


// this will print 2 things, one if the kicker is ready (without photogate)
// two: it will print if it is ready to kick with photogate, if this prints one that means it probably just kicked
// realistically, just worry about the first number as the 2nd one will only be there for like 10 loops (really quick)

// the code has been uploaded

// hold on, is this how much time is left for it to kick?

// that is quite hard as it is its own timer class, I can print if it is READY to kick, is that okay?
// 

// ok enable switch now ok

// im literally writing power to them regardless of anything, so there has tobe something wrong wiring wise

// This has photogate as well btw - so it wont kick if the photogate is not activated
// speak herew
// is bluetooth 

// that doesnt do anything im literally just writing power to dribb at 100/255

// what does that do its writing to the pin manually- you cant print nothing

// no because this is a completely seperate file on its own

// click enable switch, but shouldnt kick until ball is in capture