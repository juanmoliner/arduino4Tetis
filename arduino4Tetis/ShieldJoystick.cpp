#include "ShieldJoystick.h"


void setupShieldJoystick(){
  // Setup for the Joystick in the Sparkfun CANBus shield

    //Initialize analog pins as inputs
    pinMode(UP,INPUT);
    pinMode(DOWN,INPUT);
    pinMode(LEFT,INPUT);
    pinMode(RIGHT,INPUT);
    pinMode(CLICK,INPUT);

    //Pull analog pins high to enable reading of joystick movements
    digitalWrite(UP, HIGH);
    digitalWrite(DOWN, HIGH);
    digitalWrite(LEFT, HIGH);
    digitalWrite(RIGHT, HIGH);
    digitalWrite(CLICK, HIGH);
}

void Joystick::read(){
  // Scans Joystick analog pins, if enough time since last read has passed
  if(millis() - tLastRead > readInterval){
    tLastRead = millis();
    if (digitalRead(UP) == 0) { // joystick pressed in "up" direction
      if( y == 0 ){
        // previous read was not pressed
        tInitY = millis();
      }
      y = 1;
    }
    else if (digitalRead(DOWN) == 0) {
      if( y == 0 ){
        tInitY = millis();
      }
      y = -1;
    }
    else y = 0;
    if (digitalRead(RIGHT) == 0) {
      if( x == 0 ){
        tInitX = millis();
      }
      x = 1;
    }
    else if (digitalRead(LEFT) == 0) {
      if( x == 0 ){
        tInitX = millis();
      }
      x = -1;
    }
    else x = 0;
    if (digitalRead(CLICK) == 0) {
      if( z == 0 ){
        tInitZ = millis();
      }
      if(tInitZ)
      z = -1;
    }
    xAcc = x * Kacc * (long)(millis() - tInitX);
    yAcc = y * Kacc * (long)(millis() - tInitY);
    zAcc = z * Kacc * (long)(millis() - tInitZ);
  }
}
