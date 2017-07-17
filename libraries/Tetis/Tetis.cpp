#include "Arduino4Tetis.h"
#include "TetisKinematics.h"



void Tetis :: initPosition(){
  // initial position control in space of the joints to take them out
  // of calibration position
  float q0[4] = Q_INIT_POSITION;
  float maxError = 0; // max error out of all joints

  for(int i = 0; i < 4; i++){
    qd[i] = q0[i];
  }
  jointPosControl();

  for(int i = 0; i < 4; i++){
    if(abs(qError[i]) > maxError){        // pick the max error out of al joints
      maxError = abs(qError[i]);
    }
  }
  if(user.debugMode){
    serial -> print("DEBUG: Tetis::initPosition(): Max error[deg]: "); serial -> print(maxError * RADTODEG,4);
    serial -> print(" Max error allowed[deg] : "); serial -> println(INIT_Q_MAX_ERROR,4);
  }
  if(maxError * RADTODEG < INIT_Q_MAX_ERROR){  // initial control was fulfilled
    initialControl = true;
    if(user.debugMode){
      serial -> println("DEBUG: initQPosition(): Exitting");
    }
    // zero control for safety
    for(int i = 0; i < 4; i++){
      systemJoints[i]-> u = 0.0;
    }
  }
}

void Tetis :: updateControl(unsigned int operMode){
  float q0[4] = Q_INIT_POSITION;
  Joint* joint;
  if(!offsetRead){             // offset not set yet: 1st control loop execution
    if(user.debugMode){
      serial -> println("DEBUG: Tetis :: updateControl() entering offset setup");
    }
    for(int i = 0; i < 4; i++ ){
      joint = systemJoints[i];
      joint -> qoffset = joint -> q - joint -> qcalib;
    }
    offsetRead = true;
  }
  else if(!initialControl){   // initial position control not fulfilled yet
    initPosition();
    if(user.toMatlab){
      matlab.plotQ(this);
      matlab.plotU(this);
    }
  }
  else{                       // both initial offset setup & joint contol already made
    switch(operMode){
      case 1: //JointControl
        if(user.debugMode){
          serial -> println("DEBUG: loop(): entering joint pos control");
        }
        jointPosControl();
        if(user.toMatlab){
          matlab.plotQ(this);
          matlab.plotU(this);
        }
        break;
      case 2: // joystick in base
        if(user.debugMode){
          serial -> println("DEBUG: loop(): entering joystick base pos control");
        }
        joystickBaseControl();
        if(user.toMatlab){
          matlab.plotX(this);
          matlab.plotQ(this);
          matlab.plotU(this);
        }
        break;
      case 3 : // Joystick in actuator
        if(user.debugMode){
          serial -> println("DEBUG: loop(): entering joystick act pos control");
        }
        joystickActuatorControl();
        if(user.toMatlab){
          matlab.plotX(this);
          matlab.plotQ(this);
          matlab.plotU(this);
        }
        break;
      case 4: // trajectory
        if(user.debugMode){
          serial -> println("DEBUG: loop(): entering trajectory control");
        }
        evalTrajectory(TETIS_TRAJ);
        trajectoryControl();
        if(user.toMatlab){
          matlab.plotX(this);
          matlab.plotQ(this);
          matlab.plotU(this);
        }
        break;
      case 5: // intitial position again
        if(user.debugMode){
          serial -> println("DEBUG: loop(): take to start position");
        }
        for(int j = 0; j < 4; j++){
          qd[j] = q0[j];
        }
        jointPosControl();
        if(user.toMatlab){
          matlab.plotQ(this);
          matlab.plotU(this);
        }
        break;
      default :
        Serial.println("WARN: Tetis :: updateControl(): wrong control mode");
        Serial3.println("WARN: Tetis :: updateControl(): wrong control mode");
        break;
    }
  }
}

void Tetis :: jointPosControl(){
  // Position control in the space of the joints (position reference constant in time)
  for (int i = 0; i < numOfJoints; i++){
    // error[i] = qd[i] - q[i];
    // u[i] = kj[i] * error[i];
    qError[i] = qd[i] - systemJoints[i] -> q;
    systemJoints[i] -> u = kj * qError[i];
    if(user.debugMode){
      serial -> print("DEBUG: Tetis :: JointPosControl(): Joint: "); serial -> print(i+1);
      serial -> print(" u[rad/s] = ");serial -> print(systemJoints[i] -> u, DEBUG_PREC);
      serial -> print(" qError[rad/s] = ");serial -> print(qError[i], DEBUG_PREC);
      serial -> print(" qd[deg] = "); serial -> print(qd[i] * RADTODEG, DEBUG_PREC);
      serial -> print(" q[deg] = ");serial -> println(systemJoints[i] -> q * RADTODEG, DEBUG_PREC);
    }
  }
}

void Tetis :: evalTrajectory(unsigned int trajNum){
  float t = tLastExec / 1000.0;
  const float wn = PI / 10;

  // evaluate trajectory
  switch(trajNum){
    case 1:   // // Luis Gustavo Exp1 (6.4.1)
      xd[0] = 500;
      xd[1] = -50;
      xd[2] = -150;
      xd[3] = 0.5235;
      xddot[0] = xddot[1] = xddot[2] = xddot[3] = 0.0;
      break;
    case 2:  //Luis Gustavo Exp 6.5.1
      xd[0] = 75 * (sin(wn * t) + sin(4 * wn * t)) + 500;
      xd[1] = 57.0;
      xd[2] = 75 * (cos(wn * t) + cos(4 * wn * t)) - 67;
      xd[3] = wn * sin(wn * t);

      xddot[0] = 75 * wn * cos(wn * t) + 300 * wn * cos(4 * wn * t);
      xddot[1] = 0;
      xddot[2] = -75 * wn * sin(wn * t) - 300 * wn * sin(4 * wn * t);
      xddot[3] = wn * wn * cos(wn * t);
      break;

    case 3:   // Luis Gustavo Exp 6.5.1
      xd[0] = ((100 * cos(t)) / (sin(t) * sin(t) + 1)) + 500;
      xd[1] = 57.0;
      xd[2] = ((100 * cos(t) * sin(t)) / (sin(t) * sin(t) + 1)) - 50;
      xd[3] = 0.0;

      xddot[0] = (100 * sin(t) * (sin(t) * sin(t) - 3)) / ((sin(t) * sin(t) + 1) * (sin(t) * sin(t) + 1));
      xddot[1] = 0.0;
      xddot[2] = - (300 *  sin(t) * sin(t) - 100) / ((sin(t) * sin(t) + 1) * (sin(t) * sin(t) + 1));
      xddot[3] = 0.0;
      break;

    case 4:
      xd[0] = 5 * (16 * sin(t) * sin(t) * sin(t)) + 500;
      xd[1] = 57.0;
      xd[2] = 5 * (13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t)) - 50;
      xd[3] = 0.0;

      xddot[0] = 5 * 48 * sin(t) * sin(t) * cos(t);
      xddot[1] = 0.0;
      xddot[2] = 5 * (- 13 * sin(t) + 10 * sin(2*t) + 6 * sin(3*t) + 4* sin(4*t));
      xddot[3] = 0.0;
      break;

    }
}

void Tetis :: trajectoryControl(){
  // Proportional control with Feedforward
  bool invertMat;
  float J0_inv[4][4]; // Inverse of jacobian at the base (joint 0)
  float ubar[4];
  float auxU[4];      // auxiliary array

  updateJacob0();
  Matrix.Copy((float *)J0, 4, 4, (float *)J0_inv);
  invertMat = Matrix.Invert((float *)J0_inv, 4); // already saves the inverse mat in input mat

  if ( ! invertMat ){               // If singular matrix -> zero control
    for (int i = 0; i < 4; i++){
      systemJoints[i]-> u = 0;
    }
  }
  else{
    // e = xd(h) - x(h)
    // ubar = Kp * error + xdot(h)
    for (int i = 0; i < 4; i++){
        xError[i] = xd[i] - x[i];
        ubar[i] = kp * xError[i] + xddot[i];
    }
    // u = J0^(-1) * ubar
    Matrix.Multiply((float*)J0_inv,ubar, 4, 4, 1,auxU);
  }
  for (int i = 0; i < 4; i++){
    systemJoints[i]-> u = auxU[i];
  }

  if(user.debugMode){
    for (int i = 0; i < 4; i++){
      serial -> print("DEBUG: proportionalFF(): i = "); serial -> print(i+1);
      serial -> print(" ui[rad/s] = ");serial -> print(systemJoints[i]-> u,DEBUG_PREC);
      serial -> print(" xdi[mm] = "); serial -> print(xd[i],DEBUG_PREC);
      serial -> print(" xi[mm] = ");serial -> println(x[i],DEBUG_PREC);
    }
  }
}

void Tetis :: joystickBaseControl(){
  // Joystick position control (fixed pitch)
  bool invertMat;
  float r[4];
  float auxU[4];
  float J0_inv[4][4]; // Inverse of jacobian at the base (joint 0)
  updateJacob0();
  Matrix.Copy((float*)J0, 4, 4, (float*)J0_inv);
  invertMat = Matrix.Invert((float *)J0_inv,4); // already saves the inverse mat in input mat
  if ( ! invertMat ){
    // If singular matrix -> zero control
    for (int i = 0; i < 4; i++){
      systemJoints[i]-> u = 0;
    }
  }
  else{
    r[0] = K_JOYSTICK * joystick -> getX(); // x joystick -> x robot
    r[1] = K_JOYSTICK * joystick -> getZ(); // z joystick -> y robot
    r[2] = K_JOYSTICK * joystick -> getY(); // y joystic -> z robot
    r[3] = 0.0;
    // u = J0^(-1) * r
    Matrix.Multiply((float*)J0_inv,r, 4, 4, 1,auxU);
    for (int i = 0; i < 4; i++){
      systemJoints[i]-> u = auxU[i];
    }
  }
}

void Tetis :: joystickActuatorControl(){
  // Joystick position control (fixed pitch)
  bool invertMat;
  float r[4];
  float auxU[4];
  float JN_inv[4][4]; // Inverse of jacobian at the base (joint 0)
  updateJacobN();
  Matrix.Copy((float *)JN, 4, 4, (float *)JN_inv);
  invertMat = Matrix.Invert((float *)JN_inv,4); // already saves the inverse mat in input mat
  if ( ! invertMat ){
    // If singular matrix -> zero control
    for (int i = 0; i < 4; i++){
      systemJoints[i]-> u = 0;
    }
  }
  else{
    r[0] = K_JOYSTICK * joystick -> getX(); // x joystick -> x robot
    r[1] = K_JOYSTICK * joystick -> getZ(); // z joystick -> y robot
    r[2] = K_JOYSTICK * joystick -> getY(); // y joystic -> z robot
    r[3] = 0.0;
    // u = J0^(-1) * r
    Matrix.Multiply((float*)JN_inv,r, 4, 4, 1,auxU);
    for (int i = 0; i < 4; i++){
      systemJoints[i]-> u = auxU[i];
    }
  }
}

bool Tetis :: checkJointLimits(){
  // Tetis specific function: checks if any joint has exceeded its mechanical
  // limit (returns true if so)
    if ( (   systemJoints[0] -> q > PI/2 || systemJoints[0] -> q < -PI/2
          || systemJoints[1] -> q > 0    || systemJoints[1] -> q < -PI/2
          || systemJoints[2] -> q > PI   || systemJoints[2] -> q < -PI
          || systemJoints[3] -> q > PI   || systemJoints[3] -> q < -PI
        ) &&  initialControl
     )
    {
        Serial.println("WARN: tetischeckJointLimits(): Joints execeded limits");
        Serial3.println("WARN: tetischeckJointLimits(): Joints execeded limits");
        return true;
    }
    else return false;
}

bool Tetis :: checkCollision(){
  // Tetis specific function: checks if any joint is about to colide
  // with another (returns true if so)
  // Calculating Forward Kinematics for each joint
    float p02[3] = {E3*c1*c2, E3*c2*s1, E3*s2};
    float p23[3] = {E4*c1*c2*c3 - E4*c1*s2*s3, E4*c2*c3*s1 - E4*s1*s2*s3, c2*s3 + c3*s2};
    float p03[3];
    float p04[3] =  {-M5*s1+E4*c23*c1+E3*c1*c2+E5*c234*c1,
                      M5*c1+E4*c23*s1+E3*c2*s1+E5*c234*s1,
                      E4*s23+E3*s2+E5*s234};
    bool joint3, joint4, eff;
    joint3 = joint4 = eff = false;
    for(int i = 0; i<3; i++){
      p03[i] = p02[i] + p23[i];
    }
    if ( (p02[0] < 200) && (p02[1] < 152 || p02[1] > -152) &&  (p02[2] > 0)){
        Serial.println("WARN: tetisCheckcollision(): COLLISION imminent with joint 3");
        Serial3.println("WARN: tetisCheckcollision(): COLLISION imminent with joint 3");
        joint3 = true;
    }
    if ( (p03[0] < 200) && (p03[1] < 152 || p03[1] > -152) && (p03[2] > 0) ){
        Serial.println("WARN: tetisCheckcollision(): COLLISION imminent with joint 4");
        Serial3.println("WARN: tetisCheckcollision(): COLLISION imminent with joint 4");
        joint4 = true;
    }
    if ( (p04[0] < 200) && (p04[1] < 152 || p04[1] > -152) && (p04[2] > 0) ){
        Serial.println("WARN: tetisCheckcollision(): COLLISION imminent with effector");
        Serial.println("WARN: tetisCheckcollision(): COLLISION imminent with effector");
        eff = true;
    }

    if (joint3 || joint4 || eff){
      return true;
    }
    else return false;
}

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
