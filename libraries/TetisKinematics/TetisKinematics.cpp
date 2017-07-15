#include "Arduino4Tetis.h"
#include "TetisKinematics.h"


void Tetis :: updateDHData(){

    c1 = cos(systemJoints[0] -> q);
    s1 = sin(systemJoints[0] -> q);

    c2 = cos(systemJoints[1] -> q);
    s2 = sin(systemJoints[1] -> q);

    c3 = cos(systemJoints[2] -> q);
    s3 = sin(systemJoints[2] -> q);

    c4 = cos(systemJoints[3] -> q);
    s4 = sin(systemJoints[3] -> q);

    c23 = cos(systemJoints[1] -> q + systemJoints[2] -> q);
    s23 = sin(systemJoints[1] -> q + systemJoints[2] -> q);

    c34 = cos(systemJoints[2] -> q + systemJoints[3] -> q);
    s34 = sin(systemJoints[2] -> q + systemJoints[3] -> q);

    c234 = cos(systemJoints[1] -> q + systemJoints[2] -> q + systemJoints[3] -> q);
    s234 = sin(systemJoints[1] -> q + systemJoints[2] -> q + systemJoints[3] -> q);
}

void  Tetis :: updateDirectKinematics(){
    // Forward kinematics p = k(theta)
    updateDHData();
    x[0] = -M5*s1+E4*c23*c1+E3*c1*c2+E5*c234*c1;
    x[1] =  M5*c1+E4*c23*s1+E3*c2*s1+E5*c234*s1;
    x[2] =  E4*s23+E3*s2+E5*s234;
    x[3] = - (systemJoints[1] -> q + systemJoints[2] -> q  + systemJoints[3] -> q);
}


void Tetis :: updateJacob0(){
  // Jacobian in coordenate system of the base
    J0[0][0] = - M5*c1 - E4*c23*s1 - E3*c2*s1 - E5*c234*s1;
    J0[0][1] = -c1*(E4*s23+E3*s2+E5*s234);
    J0[0][2] = -c1*(E4*s23+E5*s234);
    J0[0][3] = -E5*s234*c1;
    J0[1][0] = -M5*s1+E4*c23*c1+E3*c1*c2+E5*c234*c1;
    J0[1][1] = -s1*(E4*s23+E3*s2+E5*s234);
    J0[1][2] = -s1*(E4*s23+E5*s234);
    J0[1][3] = -E5*s234*s1;
    J0[2][0] = 0.0;
    J0[2][1] = E4*c23+E3*c2+E5*c234;
    J0[2][2] = E4*c23+E5*c234;
    J0[2][3] = E5*c234;
    J0[3][0] = 0.0;
    J0[3][1] = -1.0;
    J0[3][2] = -1.0;
    J0[3][3] = -1.0;
}

void Tetis :: updateJacobN(){
  // Jacobian in coordenate system of the actuator
    JN[0][0] = -M5*c234;
    JN[0][1] = E3*s34+E4*s4;
    JN[0][2] = E4*s4;
    JN[0][3] = 0.0;
    JN[1][0] = E4*c23+E3*c2+E5*c234;
    JN[1][1] = 0.0;
    JN[1][2] = 0.0;
    JN[1][3] = 0.0;
    JN[2][0] = M5*s23;
    JN[2][1] = E5+E3*c34+E4*c4;
    JN[2][2] = E5+E4*c4;
    JN[2][3] = E5;
    JN[3][0] = 0.0;
    JN[3][1] = -1.0;
    JN[3][2] = -1.0;
    JN[3][3] = -1.0;
}
