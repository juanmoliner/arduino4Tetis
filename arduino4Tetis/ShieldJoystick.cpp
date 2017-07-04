#include "ShieldJoystick.h"
#include "Arduino4Tetis.h"


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

void ShieldJoystick::read(){
  // Scans Joystick analog pins, if enough time since last read has passed
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

void  BluetoothJoystick :: read(){
  int joystickRead = 0;
  if(selectModeFlag){
    BluetoothJoystick :: selectMode();
  }
  else{
    if (Serial3.available() > 0) {
     joystickRead = Serial3.read();
    }
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
        userControl = InitialPosition;
        #ifdef DEBUG_MODE
        Serial.println("DEBUG: BluetoothJoystick :: read(): take to start position");
        #endif
        break;
    }
  }
}

void BluetoothJoystick :: selectMode(){
  int joystickRead = 0;
  #ifdef DEBUG_MODE
  Serial.println("DEBUG: BluetoothJoystick :: selectMode(): entered");
  #endif
  if (Serial3.available() > 0) {
   joystickRead = Serial3.read();
   selectModeFlag = false;
  }
  switch (joystickRead){
    case 'i': // circle
      #ifdef DEBUG_MODE
      Serial.println("DEBUG: BluetoothJoystick :: selectMode(): Joystick in actuator system control selected");
      #endif
      userControl = JoystickActuator;
      break;
    case 'f': // square
      #ifdef DEBUG_MODE
      Serial.println("DEBUG: BluetoothJoystick :: selectMode(): Joystick in base system control selected");
      #endif
      userControl = JoystickBase;
      break;
    case 'g': // triangle
      #ifdef DEBUG_MODE
      Serial.println("DEBUG: BluetoothJoystick :: selectMode(): Trajectory mode selected");
      #endif
      userControl = Trajectory;
      break;
  }
 }
