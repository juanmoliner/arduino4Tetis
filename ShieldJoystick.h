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
    char x;
    char y;
    char z;
    // proportional(Kacc) to the time the button has been continuosly pressed
    long unsigned xAcc;
    long unsigned yAcc;
    long unsigned zAcc;
    int Kacc = 1;
    // millis started last click on each direction
    long unsigned tInitX;
    long unsigned tInitY;
    long unsigned tInitZ;

    long unsigned tLastRead = 0; // millis of last read
    long unsigned readInterval = 50; // interval in between reads

  public:
    void read();
    char getX(){return x;}
    char getY(){return y;}
    char getZ(){return z;}
    long getXacc(){ return xAcc;}
    long getYacc(){ return yAcc;}
    long getZacc(){ return zAcc;}

};

extern  Joystick shieldJoystick;

#endif // SHIELDJOYSTICK_H
