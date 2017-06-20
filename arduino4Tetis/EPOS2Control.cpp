#include "EPOS2Control.h"
#include "Arduino4Tetis.h"
#include "TetisKinematics.h"
#include <MatrixMath.h>
#include <math.h>

void proportionalFF(){
  // Proportional control with Feedforward
  bool invertMat;
  updateJacob0();
  Matrix.Copy((float *)J0, NUMBEROFNODES, NUMBEROFNODES, (float *)J0_inv);
  invertMat = Matrix.Invert((float *)J0_inv, NUMBEROFNODES); // already saves the inverse mat in input mat
  if ( ! invertMat ){
    // If singular matrix -> zero control
    // possib : jointsOutSingular ??
    for (int i = 0; i < NUMBEROFNODES; i++){
      u[i] = 0;
    }
  }
  else{
    // e = xd(h) - x(h)
    // ubar = Kp * error + xdot(h)
    for (int i = 0; i < NUMBEROFNODES; i++){
        error[i] = xd_h[i] - x[i];
        ubar[i] = KP * error[i] + xddot_h[i];
    }
    // u = J0^(-1) * ubar
    Matrix.Multiply((float *)J0_inv,(float *) ubar, NUMBEROFNODES, NUMBEROFNODES, 1, (float *)u);
  }
  for (int i = 0; i < NUMBEROFNODES; i++){
  // check saturation
  /* DEBUGGING PURPOSES */
  #ifdef DEBUG_MODE
    Serial.print("DEBUG: proportionalFF(): i = "); Serial.print(i+1);
    Serial.print(" : xdi = "); Serial.print(xd_h[i]);
    Serial.print(" xi = ");Serial.print(x[i]);
    Serial.print(" ui = ");Serial.println(u[i]);
  #endif
  /* END OF DEBUGGING PURPOSES */
  }
}


void trajectoryControl(){
  // Follows a predefined trajectory in the space of the actuator
  // Control: Proportional + Feedforward (space of the actuator)

  // unsigned long t = tLastExec;

  // evaluate trajectoryControl

  // Luis Gustavo Exp1 (6.4.1)
  xd_h[0] = 500;
  xd_h[1] = -50;
  xd_h[2] = -150;
  xd_h[3] = 0.5235;
  // //
  // r_h[0] = 500;
  // r_h[1] = -50;
  // r_h[2] = -150;
  // r_h[3] = 0.5235;
  // // First order filter:
  // // xd(h) = (1 - GAMMA * h) * xd(h-1) + GAMMA * h * r(h-1)
  // // xd_dot(h) = - GAMMA * xd(h) + GAMMA * r(h)
  // for(int i = 0; i < NUMBEROFNODES; i++){
  //   xd_h[i] = (1 - GAMMA * h) * xd_h_1[i] + GAMMA * h * r_h_1[i];
  //   xddot_h[i] = - GAMMA * xd_h[i] + GAMMA * r_h[i];
  //   r_h_1[i] = r_h[i]; // saves r(h) to r(h-1) for next iteration
  // }

  proportionalFF();
}

void joystickControl(){
  // Joystick position control (fixed pitch)
  // Control: Proportional + Feedforward (space of the actuator)
  shieldJoystick.read(); // reads desired position from shield's joystick
  r_h[0] = shieldJoystick.getX();
  r_h[1] = shieldJoystick.getY();
  r_h[2] = shieldJoystick.getZ();
  // First order filter:
  // xd(h) = (1 - GAMMA * h) * xd(h-1) + GAMMA * h * r(h-1)
  // xd_dot(h) = - GAMMA * xd(h) + GAMMA * r(h)
  for(int i = 0; i < NUMBEROFNODES; i++){
    xd_h[i] = (1 - GAMMA * h) * xd_h_1[i] + GAMMA * h * r_h_1[i];
    xddot_h[i] = - GAMMA * xd_h[i] + GAMMA * r_h[i];
    r_h_1[i] = r_h[i]; // saves r(h) to r(h-1) for next iteration
  }
  proportionalFF();
}


void jointPosControl(){
  // Position control in the space of the joints (position reference constant in time)
  for (int i = 0; i < NUMBEROFNODES; i++){
    error[i] = qd[i] - q[i];
    u[i] = KJ * error[i];
    /* DEBUGGING PURPOSES */
    #ifdef DEBUG_MODE
    Serial.print("DEBUG: JointPosControl(): Joint: "); Serial.print(i+1);
    Serial.print(" : qd = "); Serial.print(qd[i]);
    Serial.print(" q = ");Serial.print(q[i]);
    Serial.print(" u = ");Serial.println(u[i]);
    #endif
    /* END OF DEBUGGING PURPOSES */
  }
}
