#include "Arduino4Tetis.h"
#include "TetisKinematics.h"


void updateTetisData(){
    c1 = cos(q[0]);
    s1 = sin(q[0]);

    c2 = cos(q[1]);
    s2 = sin(q[1]);

    c3 = cos(q[2]);
    s3 = sin(q[2]);

    c4 = cos(q[3]);
    s4 = sin(q[3]);

    c23 = cos(q[1]+q[2]);
    s23 = sin(q[1]+q[2]);

    c34 = cos(q[2]+q[3]);
    s34 = sin(q[2]+q[3]);

    c234 = cos(q[1]+q[2]+q[3]);
    s234 = sin(q[1]+q[2]+q[3]);
}

void updateDirectKinematics(){
    // Forward kinematics p = k(theta)
    x[0] = -M5*s1+E4*c23*c1+E3*c1*c2+E5*c234*c1;
    x[1] =  M5*c1+E4*c23*s1+E3*c2*s1+E5*c234*s1;
    x[2] =  E4*s23+E3*s2+E5*s234;
    x[3] = - (q[1] + q[2] + q[3]);
}


void updateJacob0(){
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

void updateJacobN(){
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
