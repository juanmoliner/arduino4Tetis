#ifndef EPOS2CONTROL_H
#define EPOS2CONTROL_H

#include <mcp_can.h>
#include <Arduino.h>

void proportionalFF();
void trajectoryControl();
void joystickControl();
void jointPosControl();

#endif
