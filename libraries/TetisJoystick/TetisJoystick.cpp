#include "Arduino4Tetis.h"


void ShieldJoystick :: setup(){
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

void ShieldJoystick ::read(){
  // Scans Joystick analog pins, if enough time since last read has passed
  if (digitalRead(UP) == 0) { // joystick pressed in "up" direction
    y = 1;
  }
  else if (digitalRead(DOWN) == 0) {
    y = -1;
  }
  else y = 0;

  if (digitalRead(RIGHT) == 0) {
    x = 1;
  }
  else if (digitalRead(LEFT) == 0) {
    x = -1;
  }
  else x = 0;
  if (digitalRead(CLICK) == 0) {
    z = -1;
  }
  else z = 0;
}

void  BluetoothJoystick :: read(){
  char joystickRead;
  if(selectModeFlag){     // wait for user to select mode w/o blocking exec
    BluetoothJoystick :: selectMode();
  }
  else{
    if (Serial3.available() > 0) {
      joystickRead = Serial3.read();
      switch (joystickRead){
        case 'a': // left
          x = 1; y = z = 0 ; break;
        case 'b': // right
          x = -1; y = z = 0; break;
        case 'c': // up
          y = 1; x = z = 0; break;
        case 'd': // down
          y = -1; x = z = 0; break;
        case 'g': // triangle
          z = 1; x = y = 0; break;
        case 'i': // circle
          z = -1; x = y = 0; break;
        case 'f': // square
          x = y = z = 0; break;
        case 'A': // select
          selectModeFlag = true;
          x = y = z = 0;
          break;
        case 'B': // start
          user.controlType = 5 ;
          if(user.debugMode){
            serial -> println("DEBUG: BluetoothJoystick :: read(): take to start position");
          }
          break;
      } // switch
    }// if (Serial3.available() > 0)
  }
}

void BluetoothJoystick :: selectMode(){
  char joystickRead;
  if(user.debugMode){
    serial -> println("DEBUG: BluetoothJoystick :: selectMode(): entered");
  }
  if (Serial3.available() > 0) {      // user has selected a mode
   joystickRead = Serial3.read();
   selectModeFlag = false;
  }
  switch (joystickRead){
    case 'i': // circle
      if(user.debugMode){
        serial -> println("DEBUG: BluetoothJoystick :: selectMode(): Joystick in actuator system control selected");
      }
      user.controlType = 3;
      break;
    case 'f': // square
      if(user.debugMode){
        serial -> println("DEBUG: BluetoothJoystick :: selectMode(): Joystick in base system control selected");
      }
      user.controlType = 2;
      break;
    case 'g': // triangle
      if(user.debugMode){
        serial -> println("DEBUG: BluetoothJoystick :: selectMode(): Trajectory mode selected");
      }
      user.controlType = 4;
      break;
  }
 }
