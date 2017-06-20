#include <Arduino.h>

#include <SPI.h>
#include <MatrixMath.h>
#include <math.h>
#include <mcp_can.h>

#include "Arduino4Tetis.h"
#include "EPOS2ControlMsgUtils.h"
#include "EPOS2ControlSetup.h"
#include "ShieldJoystick.h"
#include "TetisKinematics.h"
#include "EPOS2Control.h"


long unsigned h = SAMP_TIME; // Sampling time(ms) for the control loops
long unsigned tLastExec; // time(ms)loop was last executed
long unsigned tDelay; // delay from time iteration was supposed to start

long unsigned tLastPlot; // time(ms) last plot in Matlab

float r_h[NUMBEROFNODES]; // position read at joystick at h
float r_h_1[NUMBEROFNODES]; // position read at joystick at h-1


float xd_h[NUMBEROFNODES]; // desired position of the actuator at h(space of the joints)
float xd_h_1[NUMBEROFNODES]; // desired position of the actuator at h-1(space of the joints)
float xddot_h[NUMBEROFNODES]; // UNUSED???desired velocity of the actuator at h(space of the joints)
float xddot_h_1[NUMBEROFNODES]; // desired velocity of the actuator at h-1(space of the joints)

float qd[NUMBEROFNODES]; // desired position[rad] for each joint (space of the joints)

float x[NUMBEROFNODES]; // actual position of the actuator (space of the actuator)

float q[NUMBEROFNODES]; // actual position[rad] of each joint(space of the joints)
float qdot[NUMBEROFNODES]; // actual velocity [rad/s] of each joint(space of the joints)

float u[NUMBEROFNODES]; // control variable angular velocity [rad/s]
float ubar[NUMBEROFNODES]; // auxiliary control variable angular velocity [rad/s]
float error[NUMBEROFNODES]; // error  defined as: e = xd -x

float qoffset[NUMBEROFNODES]; // initial read of incremental encoder
float qinit[NUMBEROFNODES] = JOINTS_INIT_VALS;  // initial angle of each joint. MAKE SURE ALL JOINTS START IN THIS POSITION

float c1, s1, c2, s2, c3, s3, c4, s4, c23, s23, c34, s34, c234, s234; // Tetis specific variables
float J0[NUMBEROFNODES][NUMBEROFNODES]; // Jacobian at the base (joint 0)
float J0_inv[NUMBEROFNODES][NUMBEROFNODES]; // Inverse of jacobian at the base (joint 0)
float JN[NUMBEROFNODES][NUMBEROFNODES]; // Jacobian at the actuator (joint n)


Joystick shieldJoystick;

byte SYNC[2] ={0x00, 0x00};
byte OPERATIONAL[2] ={0x01, 0};
byte PREOPERATIONAL[2] ={0x80, 0};

byte SHUTDOWN[8] ={0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
byte FAULTRESET[8] ={0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
byte ONANDENABLE[8] ={0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};

MCP_CAN CAN(SPI_CS_PIN);

enum ControlType
{
  JointControl,
  Cartesian,
  ProportionalFF
};
ControlType ControlType;



void uSet(){
  // writes to EPOS the values stored in control variable u (via SDO)
  /* TESTING PURPOSES*/
  #ifdef TWO_MOTOR_TEST
  unsigned int numNodes = 2;
  #else
  unsigned int numNodes = NUMBEROFNODES;
  #endif
  /* END OF TESTING PURPOSES*/
  byte SETRPM[8] = {0x23, 0x6B, 0x20, 0x00,0,0,0,0};;
  long rpm;
  #ifndef FORGET_JLMITS_COLIS
  // Check inminet collision & joint limits
  if(tetisCheckColision() || tetisCheckJointLimits()){
    // danger: set control to zero
    for(int i = 0; i < NUMBEROFNODES; i++){
      u[i] = 0.0;
    }
  }
  #endif

  for(unsigned int nodeNum = 1 ; nodeNum <= numNodes ; nodeNum++){
    // check saturation
    if (u[nodeNum - 1] * RADSTORPM > MAX_VELOCITY){
      // if control > max veloc -> saturate output
      #ifndef FORGET_SATURATION
      /* DEBUGGING PURPOSES */
      #ifdef DEBUG_MODE
      Serial.print("DEBUG: uSet(): joint "); Serial.print(nodeNum);
      Serial.print(" saturated. u[i] = "); Serial.println(u[nodeNum - 1]);
      #endif
      /* END OF DEBUGGING PURPOSES */
      u[nodeNum - 1] = MAX_VELOCITY / RADSTORPM;
    }
    else if( u[nodeNum - 1] * RADSTORPM < -MAX_VELOCITY ){
      // if control > -max veloc -> saturate output
      /* DEBUGGING PURPOSES */
      #ifdef DEBUG_MODE
      Serial.print("DEBUG: uSet(): joint "); Serial.print(nodeNum);
      Serial.print(" saturated. u[i] = "); Serial.println(u[nodeNum - 1]);
      #endif
      /* END OF DEBUGGING PURPOSES */
      u[nodeNum - 1] = - (MAX_VELOCITY / RADSTORPM);
    }
    #endif
    // set rpm to EPOS
    rpm = u[nodeNum - 1] * RADSTORPM;
    byte* rpmBytes = (byte*) &rpm; // to be able to acces value byte by byte
    for(int i = 0; i < 4; i++){
      SETRPM[i + 4] = rpmBytes[i];
    }
    CAN.sendMsgBuf(0x600 + nodeNum,0,8,SETRPM);
    printMsgCheck();
  }
}

inline void setVelocity(int nodeNum, long rpm){
  // sets to EPOS the target velocity for joint nodeNum passed in rpm (via SDO)
    word nodeID = 0x600 + nodeNum;
    byte* rpmBytes = (byte*) &rpm; // to be able to acces value byte by byte
    byte SETRPM[8] = {0x23, 0x6B, 0x20, 0x00, rpmBytes[0], rpmBytes[1], rpmBytes[2], rpmBytes[3]};
    CAN.sendMsgBuf(nodeID,0,8,SETRPM);
    printMsgCheck();
  }



  bool tetisCheckColision(){
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
          Serial.println("WARN: tetisCheckColision(): COLLISION imminent with joint 3");
          joint3 = true;
      }
      if ( (p03[0] < 200) && (p03[1] < 152 || p03[1] > -152) && (p03[2] > 0) ){
          Serial.println("WARN: tetisCheckColision(): COLLISION imminent with joint 4");
          joint4 = true;
      }
      if ( (p04[0] < 200) && (p04[1] < 152 || p04[1] > -152) && (p04[2] > 0) ){
          Serial.println("WARN: tetisCheckColision(): COLLISION imminent with effector");
          eff = true;
      }

      if (joint3 || joint4 || eff){
        return true;
      }
      else return false;
  }

bool tetisCheckJointLimits(){
  // Tetis specific function: checks if any joint has exceeded its mechanical
  // limit (returns true if so)
      if ( q[1] > 0 || q[1] < -PI/2 || q[2] > PI || q[2] < -PI || q[3] > PI || q[3] < -PI )
      {
          Serial.println("WARN: tetisCheckJointLimits(): Joints execeded limits");
          return true;
      }
      else return false;
  }

inline void readTPDO1(word CANID){
  // reads TPDO1 and saves it to q and qdot
    word nodeNum = CANID - 0x180;
    float velInRpm, posInRad;
    long unsigned posInQuadCounts;
    long* auxPointer1 =  (long*) buf;
    velInRpm = (float)(*auxPointer1) / MOTOR_REDUCTION ;
    long unsigned* auxPointer2 =  (long unsigned*) (buf + 4);
    posInQuadCounts = *auxPointer2;
    qdot[nodeNum - 1] = velInRpm / RADSTORPM;
    posInRad = (float)posInQuadCounts * QDTORAD / MOTOR_REDUCTION;
    q[nodeNum - 1] = posInRad - qoffset[nodeNum - 1] + qinit[nodeNum - 1];

    /*  DEBUGGING PURPOSES */
    // #ifdef DEBUG_MODE
    // Serial.print("RPM of node "); Serial.print(nodeNum);Serial.print(": ");
    // Serial.print(velInRpm,10); Serial.println();
    // Serial.print("rad/s of node "); Serial.print(nodeNum);Serial.print(": ");
    // Serial.print(qdot[nodeNum - 1],10); Serial.println();
    // Serial.print("Postion[quadCounts](absolute) of node "); Serial.print(nodeNum);Serial.print(": ");
    // Serial.print(posInQuadCounts); Serial.println();
    // Serial.print("Offset[Rad] of node "); Serial.print(nodeNum);Serial.print(": ");
    // Serial.print(qoffset[nodeNum - 1]); Serial.println();
    // Serial.print("Position[Rad](absolute) of node "); Serial.print(nodeNum);Serial.print(": ");
    // Serial.print(posInRad); Serial.println();
    // Serial.print("Position[Rad](relative) of node "); Serial.print(nodeNum);Serial.print(": ");
    // Serial.print(q[nodeNum - 1]); Serial.println();
    // Serial.print("Control variable u[Rad] of node "); Serial.print(nodeNum); Serial.print(": ");
    // Serial.print(u[nodeNum - 1]); Serial.println();
    // #endif
    /*  END OF DEBUGGING PURPOSES */
}

void CANListener(){
  /* TESTING PURPOSES */
  #ifdef TWO_MOTOR_TEST
  unsigned char nodesRemaining = 2;
  for(int i = 2; i < NUMBEROFNODES; i++){
    u[i] = KP_SIMU * u[i]; // adjust the simulated system
    q[i] = q[i] + h * 0.001 * u[i]; // h[ms] assume EPOS respond as perfect integrator
    qdot[i] = u[i];
  }
  #else
  unsigned char nodesRemaining = NUMBEROFNODES;
  #endif
  /* END OF TESTING PURPOSES */
  word CANID;
  CAN.sendMsgBuf(0x80,0,2,SYNC); // sends Sync object
  do{ // read buffer until all nodes have responded to the sync
    CAN.readMsgBuf(&len, buf);
    CANID = CAN.getCanId();  // read data,  len: data length, buf: data buf
    if(CANID == 0x80){/* Sync objetc sucessfuly sent */}
    else if( CANID > 0x180 && CANID <= 0x180 + NUMBEROFNODES ){
      nodesRemaining--;
      readTPDO1(CANID);
    }
    else{
      // other message found, we print it
      #ifdef DEBUG_MODE
      Serial.println("DEBUG: CANListener(): Message different from TPDO1 found:");
      Serial.print("DEBUG: CANListener(): 0x");Serial.print(CAN.getCanId(),HEX);Serial.print("\t");
      for(int i = 0; i<len; i++){
        Serial.print("0x");Serial.print(buf[i],HEX);Serial.print("\t");
      }
      Serial.println();
      #endif
    }
  }while(nodesRemaining > 0);
}

void plotXInMatlab(){
  if(millis() - tLastPlot > MATLAB_PLOT_SAMPLE_T){
    Serial.print("ToMatlab:\t");
    for(int i = 0; i < NUMBEROFNODES; i++){
      Serial.print(xd_h[i]); Serial.print("\t");
    }
    for(int i = 0; i < NUMBEROFNODES; i++){
      Serial.print(u[i]); Serial.print("\t");
    }
    for(int i = 0; i < NUMBEROFNODES; i++){
      Serial.print(x[i]); Serial.print("\t");
    }
    Serial.println();
    tLastPlot = millis();
  }
}

void plotQInMatlab(){
  if(millis() - tLastPlot > MATLAB_PLOT_SAMPLE_T){
    Serial.print("ToMatlab:\t");
    for(int i = 0; i < NUMBEROFNODES; i++){
      Serial.print(qd[i]); Serial.print("\t");
    }
    for(int i = 0; i < NUMBEROFNODES; i++){
      Serial.print(u[i]); Serial.print("\t");
    }
    for(int i = 0; i < NUMBEROFNODES; i++){
      Serial.print(q[i]); Serial.print("\t");
    }
    Serial.println();
    tLastPlot = millis();
  }
}

void setup()
{
    Serial.begin(115200); // init serial comunication (USB->Arduino)
    Serial.println("--------------------------------------");
    while (CAN_OK != CAN.begin(CAN_500KBPS))     // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
    setupShieldJoystick(); // init joystick on the shield
    toAllNodesSDO(FAULTRESET,0); //Clear all errors in all nodes
    CAN.sendMsgBuf(0x000,0,2,PREOPERATIONAL); printMsgCheck(); // NMT: set CanOpen network to Pre-operational
    setInitialVals();
    setupTPDOs();
    setupVelocityMode();
    toAllNodesSDO(SHUTDOWN,0); // Send Shutdown command to all nodes
    toAllNodesSDO(ONANDENABLE,0); // Send Switch On & Enable operation command to all nodes
    CAN.sendMsgBuf(0x000,0,2,OPERATIONAL); printMsgCheck(); // NMT: set CanOpen to Operational

    delay(500); //let al the SDOs be attended

    jointsOutSingular();// take the joints out of singular position

    ControlType = ProportionalFF;
    // initXPosition(); // take actuator to an initial postition

    // ControlType = JointControl;
    // for(int i = 0; i < NUMBEROFNODES; i++){
    //   qd[i] = PI;
    // }

    // for(int i = 0; i < NUMBEROFNODES; i++){
    //   u[i] = 500.0 / RADSTORPM;
    // }
    // uSet();
    // delay(500);

    // #ifdef TWO_MOTOR_TEST
    // q[3] = PI*0.99;
    // q[2] = PI*0.99;
    // #endif
}


void loop()
{

  if((millis() - tLastExec >= h)){
    tDelay = millis() - tLastExec - h;
    if(tDelay > PERMT_DELAY && tLastExec != 0){
      Serial.print("WARN: Iteration delayed by: "); Serial.print(tDelay);Serial.println(" ms");
    }
    tLastExec = millis();
    CANListener(); // get data from EPOS nodes in CAN bus
    updateTetisData(); // update tetis values (cij, cijk,..)
    // if(tetisCheckColision() || tetisCheckJointLimits()){
    //   // danger: set control to zero
    //   for(int i = 0; i < NUMBEROFNODES; i++){
    //     u[i] = 0.0;
    //   }
    // }
    if (false){} // take off when uncomment the if avobe
    else{
      // calculate control
      switch (ControlType){
        case JointControl :
          #ifdef DEBUG_MODE
          Serial.println("DEBUG: loop(): entering joint pos control");
          #endif
          jointPosControl();
          break;
        case Cartesian :
          #ifdef DEBUG_MODE
          Serial.println("DEBUG: loop(): entering cartesian control");
          #endif
          updateDirectKinematics();
          joystickControl();
          break;
        case ProportionalFF :
          #ifdef DEBUG_MODE
          Serial.println("DEBUG: loop(): entering trajectory mode control");
          #endif
          updateDirectKinematics();
          trajectoryControl();
          break;
        default :
          Serial.println("DEBUG: loop(): Type of control invalid or not specifed");
      }
    }
    uSet(); // send control signal to nodes

    // plotQInMatlab();
    plotXInMatlab();
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
