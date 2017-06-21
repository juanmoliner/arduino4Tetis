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
        ubar[i] = kp[i] * error[i] + xddot_h[i];
    }
    // u = J0^(-1) * ubar
    Matrix.Multiply((float *)J0_inv,(float *) ubar, NUMBEROFNODES, NUMBEROFNODES, 1, (float *)u);
  }

  /* DEBUGGING PURPOSES */
  #ifdef DEBUG_MODE
  for (int i = 0; i < NUMBEROFNODES; i++){
    Serial.print("DEBUG: proportionalFF(): i = "); Serial.println(i+1);
    Serial.print("DEBUG: proportionalFF(): : xdi[mm] = "); Serial.println(xd_h[i],DEBUG_PREC);
    Serial.print("DEBUG: proportionalFF(): xi[mm] = ");Serial.println(x[i],DEBUG_PREC);
    Serial.print("DEBUG: proportionalFF(): ui[rad/s] = ");Serial.println(u[i],DEBUG_PREC);
  }
  #endif
  /* END OF DEBUGGING PURPOSES */
}


void trajectoryControl(){
  // Follows a predefined trajectory in the space of the actuator
  // Control: Proportional + Feedforward (space of the actuator)

  // unsigned long t = tLastExec;

  // evaluate trajectory

  // Luis Gustavo Exp1 (6.4.1)
  xd_h[0] = 500;
  xd_h[1] = -50;
  xd_h[2] = -150;
  xd_h[3] = 0.5235;
  xddot_h[0] = xddot_h[1] = xddot_h[2] = xddot_h[3] = 0.0;
  //


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
    u[i] = kj[i] * error[i];
    /* DEBUGGING PURPOSES */
    #ifdef DEBUG_MODE
    Serial.print("DEBUG: JointPosControl(): Joint: "); Serial.println(i+1);
    Serial.print("DEBUG: JointPosControl(): qd[deg] = "); Serial.println(qd[i] * RADTODEG, DEBUG_PREC);
    Serial.print("DEBUG: JointPosControl(): q[deg] = ");Serial.println(q[i] * RADTODEG, DEBUG_PREC);
    Serial.print("DEBUG: JointPosControl(): u[rad/s] = ");Serial.println(u[i], DEBUG_PREC);
    #endif
    /* END OF DEBUGGING PURPOSES */
  }
}
