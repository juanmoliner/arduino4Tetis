#ifndef SHIELDJOYSTICK_H
#define SHIELDJOYSTICK_H

#include <Arduino.h>
#include "Arduino4Tetis.h"

/* Define Shield Joystick connection pins */
#define UP     A1
#define DOWN   A3
#define LEFT   A2
#define RIGHT  A5
#define CLICK  A4

void setupShieldJoystick();

class Joystick
{
    // x/y/z == 1 -> Right/Up/ NOT POSSIBLE
    // x/y/z == -1 -> Left/Down/Click
    // x/y/z == 0 -> Not pressed
    protected:
      int x;
      int y;
      int z;

  public:
    virtual void read() = 0;
    virtual void setup() = 0;
    int getX(){return x;}
    int getY(){return y;}
    int getZ(){return z;}

};

class ShieldJoystick: public Joystick{
  public:
  void read();
  void setup();
 };

class BluetoothJoystick: public Joystick{
  bool selectModeFlag = false;
  public:
    void setup(){};
    void read();
    void selectMode();
 };



#endif // SHIELDJOYSTICK_H
