#ifndef EPOS2CONTROLSETUP_H
#define EPOS2CONTROLSETUP_H

#include <mcp_can.h>
#include <Arduino.h>

void setupPDOs();
void setupTPDO1();
void setupHearbeat();
void setupVelocityMode();
void setInitialVals();
void initQPosition();
void initXPosition();

#endif // EPOS2CONTROLSETUP_H
