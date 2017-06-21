#ifndef EPOS2CONTROLMESSAGEUTILS_H
#define EPOS2CONTROLMESSAGEUTILS_H
#include "Arduino.h"


void printMsgCheck();
void printMsgInterrupt();
void toAllNodesSDO(byte* , bool);

extern byte flagRecv; // Only necessary if printMsgInterrupt() is used
extern byte len;
extern byte buf[8];
extern word COBId;

#endif  // EPOS2CONTROLMESSAGEUTILS_H
