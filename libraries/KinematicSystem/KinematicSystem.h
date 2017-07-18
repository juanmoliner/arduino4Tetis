//KinematicSystem.h

#ifndef KINEMATICSYSTEM_H
#define KINEMATICSYSTEM_H

#include "Arduino4Tetis.h"

class Joint {
  public:
    bool simulated;
    bool synced;
    float q = 0;
    float qoffset = 0;
    int eposPolarity;
    long unsigned tLastHearbeat;
    unsigned long maxVel;
    unsigned long maxAcc;
    float qcalib;
    int gearRed;
    unsigned int encCpr;
    float u = 0;
    int nodeID;
    Joint(int ndId,int gr,float mv,float ma,float qc,unsigned int cpr, int pl, bool sim)
    : nodeID(ndId),gearRed(gr),qcalib(qc), encCpr(cpr),
    maxVel(mv),maxAcc(ma),eposPolarity(pl),simulated(sim){}

 };

class KinematicSystem {
  public:
    String name;
    unsigned int numOfJoints;
    bool initialControl = false;
    bool offsetRead = false;
    Joint** systemJoints;
    float xd[4];      // desired position of the actuator at (space of the joints)
    float xddot[4];   // desired velocity of the actuator at (space of the joints)
    float x[4];       // actual position of the actuator (operational space)
    float xError[4];  // error in operational space
    float qd[4];      // desired position[rad] for each joint (space of the joints)
    float qError[4];  // error in joint space
    virtual void updateControl(unsigned int operMode) = 0;
    virtual void updateDirectKinematics() = 0;
    virtual bool checkCollision() = 0;
    virtual bool checkJointLimits() = 0;
    KinematicSystem(Joint** sj,unsigned int numJ) : systemJoints(sj), numOfJoints(numJ){}
 };

 #endif
