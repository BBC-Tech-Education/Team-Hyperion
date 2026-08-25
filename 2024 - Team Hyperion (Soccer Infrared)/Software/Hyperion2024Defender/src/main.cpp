//Include init.h
#include <init.h>

//Run through once (Arduino Setup)
void setup()
{
  ls.init(); //Initializing Light Sensors
  tssp.init(); //Initializing Tssps
  motors.init(); //Intializing Drive System
  compass.setExtCrystalUse(true); //Initializing BNO
  Serial.begin(9600); //Intializing Serial
  while(!compass.begin()) {
    Serial.println("NOOOOOOOOOOOoo bno ded ;("); //Checking if BNO dies ;((
  } 
}

//Run through continously (Arduino Loop)
void loop() 
{
  compass.getEvent(&direction); //getting direction through BNo (direction.orientation.x)
  mainthings.compassVal = direction.orientation.x;
  mainthings.ballDirection = tssp.read();
  mainthings.tsspValue = tssp.tsspStrength;
  if (mainthings.compassVal < CIRCLE_DEGREES && mainthings.compassVal > SEMI_CIRCLE_DEGREES) {
    mainthings.compassVal -= CIRCLE_DEGREES;
  }
  motors.run_all(orbit.calculate_Defense_Speed(tssp.tsspStrength), orbit.calculate_Defense_Direction(mainthings.ballDirection), mainthings.ballDirection);
}