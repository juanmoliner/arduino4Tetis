//Tetis.h

#ifndef TETIS_H
#define TETIS_H

#include <mcp_can.h>
#include <Arduino.h>
#include "Arduino4Tetis.h"


class Tetis : public KinematicSystem {
    float c1, s1, c2, s2, c3, s3, c4, s4, c23, s23, c34, s34, c234, s234; // Tetis specific variables
    float J0[4][4]; // Jacobian at the base (joint 0)
    float JN[4][4]; // Jacobian at the actuator (joint n)
    float kj = 1; // Joint control gain
    float kp = 1; // proportionalFF control gain
   public:
    Tetis(Joint** sj) : KinematicSystem(sj,4){
      name = "Tetis";
    }
    void updateControl(unsigned int operMode);
    void updateJacob0();
    void updateJacobN();
    void updateDirectKinematics();
    void updateDHData();
    void initPosition();
    void jointPosControl();
    void evalTrajectory(unsigned int);
    void trajectoryControl();
    void joystickActuatorControl();
    void joystickBaseControl();
    bool checkCollision();
    bool checkJointLimits();
 };


#endif //TETISKINEMATICS_H
