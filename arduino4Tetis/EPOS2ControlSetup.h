#ifndef EPOS2CONTROLSETUP_H
#define EPOS2CONTROLSETUP_H

#include <mcp_can.h>
#include <Arduino.h>

void setupTPDOs();
void setupVelocityMode();
void setInitialVals();
void jointsOutSingular();
void initXPosition();

#endif // EPOS2CONTROLSETUP_H
