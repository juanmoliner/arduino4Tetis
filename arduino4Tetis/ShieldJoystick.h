#ifndef SHIELDJOYSTICK_H
#define SHIELDJOYSTICK_H

#include <Arduino.h>

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
      // proportional(Kacc) to the time the button has been continuosly pressed
      long unsigned xAcc;
      long unsigned yAcc;
      long unsigned zAcc;
      int Kacc = 1;
      // millis started last click on each direction
      long unsigned tInitX;
      long unsigned tInitY;
      long unsigned tInitZ;

  public:
    virtual void read() = 0;
    int getX(){return x;}
    int getY(){return y;}
    int getZ(){return z;}
    long getXacc(){ return xAcc;}
    long getYacc(){ return yAcc;}
    long getZacc(){ return zAcc;}

};

class ShieldJoystick: public Joystick{
  public:
  void read();
 };

class BluetoothJoystick: public Joystick{
  public:
    void read();
    void selectMode();
 };



#endif // SHIELDJOYSTICK_H
