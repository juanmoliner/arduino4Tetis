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
    Serial.print("DEBUG: proportionalFF(): i = "); Serial.print(i+1);
    Serial.print(" ui[rad/s] = ");Serial.print(u[i],DEBUG_PREC);
    Serial.print(" xdi[mm] = "); Serial.print(xd_h[i],DEBUG_PREC);
    Serial.print(" xi[mm] = ");Serial.println(x[i],DEBUG_PREC);
  }
  #endif
  /* END OF DEBUGGING PURPOSES */
}


void trajectoryControl(){
  // Follows a predefined trajectory in the space of the actuator
  // Control: Proportional + Feedforward (space of the actuator)

  float t = tLastExec / 1000.0;
  const float wn = PI / 10;

  // evaluate trajectory

  // Luis Gustavo Exp1 (6.4.1)
  // xd_h[0] = 500;
  // xd_h[1] = -50;
  // xd_h[2] = -150;
  // xd_h[3] = 0.5235;
  // xddot_h[0] = xddot_h[1] = xddot_h[2] = xddot_h[3] = 0.0;

  // //Luis Gustavo Exp 6.5.1
  // xd_h[0] = 75 * (sin(wn * t) + sin(4 * wn * t)) + 500;
  // xd_h[1] = 57.0;
  // xd_h[2] = 75 * (cos(wn * t) + cos(4 * wn * t)) - 67;
  // xd_h[3] = wn * sin(wn * t);
  //
  // xddot_h[0] = 75 * wn * cos(wn * t) + 300 * wn * cos(4 * wn * t);
  // xddot_h[1] = 0;
  // xddot_h[2] = -75 * wn * sin(wn * t) - 300 * wn * sin(4 * wn * t);
  // xddot_h[3] = wn * wn * cos(wn * t);

  // Luis Gustavo Exp 6.5.1
  xd_h[0] = ((100 * cos(t)) / (sin(t) * sin(t) + 1)) + 500;
  xd_h[1] = 57.0;
  xd_h[2] = ((100 * cos(t) * sin(t)) / (sin(t) * sin(t) + 1)) - 50;
  xd_h[3] = 0.0;

  xddot_h[0] = (100 * sin(t) * (sin(t) * sin(t) - 3)) / ((sin(t) * sin(t) + 1) * (sin(t) * sin(t) + 1));
  xddot_h[1] = 0.0;
  xddot_h[2] = - (300 *  sin(t) * sin(t) - 100) / ((sin(t) * sin(t) + 1) * (sin(t) * sin(t) + 1));
  xddot_h[3] = 0.0;


  proportionalFF();
}

void joystickControl(){
  // Joystick position control (fixed pitch)
  // Control: Proportional + Feedforward (space of the actuator)
  shieldJoystick.read(); // reads desired position from shield's joystick
  r_h[0] = shieldJoystick.getX(); // x joystick -> x robot
  r_h[2] = shieldJoystick.getY(); // y joystic -> z robot
  // r_h[2] = shieldJoystick.getZ();
  r_h[1] = r_h[3] = 0.0;

  #ifdef DEBUG_MODE
  Serial.print("DEBUG: joystickControl(): r_h[0] = "); Serial.print(r_h[0]);
  Serial.print(" r_h[1] = ");Serial.println(r_h[1]);
  #endif

  // First order filter:
  // xd(h) = (1 - GAMMA * h) * xd(h-1) + GAMMA * h * r(h-1)
  // xd_dot(h) = - GAMMA * xd(h) + GAMMA * r(h)
  for(int i = 0; i < NUMBEROFNODES; i++){
    xd_h[i] = (1 - GAMMA * h / 1000.0) * xd_h_1[i] + GAMMA * h / 1000.0 * r_h_1[i];
    xddot_h[i] = - GAMMA * xd_h[i] + GAMMA * r_h[i];
    r_h_1[i] = r_h[i]; // saves r(h) to r(h-1) for next iteration
    xd_h_1[i] = xd_h[i];
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
    Serial.print("DEBUG: JointPosControl(): Joint: "); Serial.print(i+1);
    Serial.print(" u[rad/s] = ");Serial.print(u[i], DEBUG_PREC);
    Serial.print(" qd[deg] = "); Serial.print(qd[i] * RADTODEG, DEBUG_PREC);
    Serial.print(" q[deg] = ");Serial.println(q[i] * RADTODEG, DEBUG_PREC);
    #endif
    /* END OF DEBUGGING PURPOSES */
  }
}
