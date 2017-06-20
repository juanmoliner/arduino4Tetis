#ifndef TETISKINEMATICS_H
#define TETISKINEMATICS_H

#include <mcp_can.h>
#include <Arduino.h>

void updateTetisData();
void updateDirectKinematics();
void updateJacob0();
void updateJacobN();


#endif //TETISKINEMATICS_H
