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
#include "MatlabSerial.h"


long unsigned h = SAMP_TIME; // Sampling time(ms) for the control loops
long unsigned tLastExec; // time(ms)loop was last executed
long unsigned tDelay; // delay from time iteration was supposed to start

long unsigned lastHeartbeat[NUMBER_OF_JOINTS]; // last time[ms] hearbeat message was received

bool initialControl = false; // wether inital position control has already been made

long unsigned tInitPlot; // time(ms) to set as zero in Matlab plot

float r_h[NUMBER_OF_JOINTS]; // position read at joystick at h
float r_h_1[NUMBER_OF_JOINTS]; // position read at joystick at h-1

unsigned int nodeIDMapping[NUMBER_OF_JOINTS] = NODEID_MAPPING; // Node id correspondig to each joint

char eposPolarity[NUMBER_OF_JOINTS] = EPOS_POLARITY; // 1 if positive theta is hourly
unsigned int motorReduction[NUMBER_OF_JOINTS] = MOTOR_REDUCTION;
float maxVelocity[NUMBER_OF_JOINTS] = MAX_VELOCITY; // velocity limit move [rpm @ motor]
float maxAcceleration[NUMBER_OF_JOINTS] =  MAX_ACCELERATION; // max acceleration [rpm/s @ motor]


float kp[NUMBER_OF_JOINTS] = KP; // actuator control proportional gain
float xd_h[NUMBER_OF_JOINTS]; // desired position of the actuator at h(space of the joints)
float xd_h_1[NUMBER_OF_JOINTS]; // desired position of the actuator at h-1(space of the joints)
float xddot_h[NUMBER_OF_JOINTS]; // UNUSED???desired velocity of the actuator at h(space of the joints)
float xddot_h_1[NUMBER_OF_JOINTS]; // desired velocity of the actuator at h-1(space of the joints)


float qd[NUMBER_OF_JOINTS]; // desired position[rad] for each joint (space of the joints)

float x[NUMBER_OF_JOINTS]; // actual position of the actuator (space of the actuator)

float kj[NUMBER_OF_JOINTS] = KJ; // joint control proportional gain
float q[NUMBER_OF_JOINTS]; // actual position[rad] of each joint(space of the joints)

float u[NUMBER_OF_JOINTS]; // control variable angular velocity [rad/s]
float ubar[NUMBER_OF_JOINTS]; // auxiliary control variable angular velocity [rad/s]
float error[NUMBER_OF_JOINTS]; // error  defined as: e = xd -x

float q0[NUMBER_OF_JOINTS] = Q_INIT_POSITION; // initial pos[rad] joints in initial joint control (space of the joints)
long  qEncOffset[NUMBER_OF_JOINTS]; // initial read [rad] of incremental encoder
float qinit[NUMBER_OF_JOINTS] = JOINTS_INIT_VALS;  // initial(calibration) angle of each joint. MAKE SURE ALL JOINTS START IN THIS POSITION

float qoffset[NUMBER_OF_JOINTS];

float c1, s1, c2, s2, c3, s3, c4, s4, c23, s23, c34, s34, c234, s234; // Tetis specific variables
float J0[NUMBER_OF_JOINTS][NUMBER_OF_JOINTS]; // Jacobian at the base (joint 0)
float J0_inv[NUMBER_OF_JOINTS][NUMBER_OF_JOINTS]; // Inverse of jacobian at the base (joint 0)
float JN[NUMBER_OF_JOINTS][NUMBER_OF_JOINTS]; // Jacobian at the actuator (joint n)


Joystick shieldJoystick;

byte SYNC[2] = {0x00, 0x00};
byte OPERATIONAL[2] = {0x01, 0};
byte PREOPERATIONAL[2] ={ 0x80, 0};

byte DISABLEVOLTAGE[8] = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
byte SHUTDOWN[8] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
byte FAULTRESET[8] = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
byte ONANDENABLE[8] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};

MCP_CAN CAN(SPI_CS_PIN);

enum ControlType
{
  Setup,
  InitialPosition,
  JointControl,
  Joystick,
  Trajectory
};
ControlType controlType;


void uSet(){
  // writes to EPOS the values stored in control variable u (via SDO)
  unsigned int numJoints = NUMBER_OF_JOINTS;
  unsigned int nodeNum;
  long rpm;
  /* TESTING PURPOSES*/
  #ifdef TWO_MOTOR_TEST
  numJoints = 2;
  #endif
  #ifdef SIMU_MODE
  numJoints = 0;
  #endif
  /* END OF TESTING PURPOSES*/

  // check inminent collision & joint limits
  #ifndef FORGET_JLMITS_COLIS
  if(tetisCheckColision() || tetisCheckJointLimits()){
    // danger: set control to zero
    for(int i = 0; i < NUMBER_OF_JOINTS; i++){
      u[i] = 0.0;
    }
  }
  #endif

  // check saturation
  #ifndef FORGET_SATURATION
  for(unsigned int jointNum = 1 ; jointNum <= NUMBER_OF_JOINTS ; jointNum++){
    if (u[jointNum - 1] * RADSTORPM * motorReduction[jointNum - 1] > maxVelocity[jointNum - 1]){
      // if control > max veloc -> saturate output
      /* DEBUGGING PURPOSES */
      #ifdef DEBUG_MODE
      Serial.print("DEBUG: uSet(): joint "); Serial.print(jointNum);
      Serial.print(" saturated. u[rad/s] = "); Serial.print(u[jointNum - 1]);
      Serial.print(" u[rpm] = "); Serial.print(u[jointNum - 1] * RADSTORPM);
      Serial.print(" set to[rpm]:   "); Serial.println(maxVelocity[jointNum - 1] / RADSTORPM / motorReduction[jointNum - 1]);
      #endif
      /* END OF DEBUGGING PURPOSES */
      u[jointNum - 1] = maxVelocity[jointNum - 1] * RADSTORPM / motorReduction[jointNum - 1];
    }
    else if( u[jointNum - 1] * RADSTORPM * motorReduction[jointNum - 1] < - maxVelocity[jointNum - 1]){
      // if control > -max veloc -> saturate output
      /* DEBUGGING PURPOSES */
      #ifdef DEBUG_MODE
      Serial.print("DEBUG: uSet(): joint "); Serial.print(jointNum);
      Serial.print(" saturated. u[rad/s] = "); Serial.print(u[jointNum - 1]);
      Serial.print(" u[rpm] = "); Serial.print(u[jointNum - 1] * RADSTORPM);
      Serial.print(" set to[rpm]:   "); Serial.println( - maxVelocity[jointNum - 1] * RADSTORPM / motorReduction[jointNum - 1]);
      #endif
      /* END OF DEBUGGING PURPOSES */
      u[jointNum - 1] = - (maxVelocity[jointNum - 1] * RADSTORPM / motorReduction[jointNum - 1]);
    }
  }
    #endif // #ifndef FORGET_SATURATION

  for(unsigned int jointNum = 1 ; jointNum <= numJoints ; jointNum++){
    // set rpm to EPOS
    nodeNum = nodeIDMapping[jointNum - 1];
    rpm = u[jointNum - 1] * RADSTORPM * motorReduction[jointNum - 1] * eposPolarity[jointNum - 1];
    CAN.sendMsgBuf(0x200 + nodeNum,0,4,(byte*)&rpm);
  }
}


void readTPDO1(word CANID){
  // reads TPDO1 and saves it to q
    word nodeNum = CANID - 0x180;
    word jointNum;
    long posInQuadCounts;
    long* auxPointer ;

    // cutrez, tipo diccionario
    for(int i = 0; i < NUMBER_OF_JOINTS; i++){
      if(nodeIDMapping[i] == nodeNum){
        jointNum = i + 1;
      }
    }

    auxPointer =  (long*) buf; //access last 4 bytes of TPDO data
    posInQuadCounts = *auxPointer;

    q[jointNum - 1] = (posInQuadCounts * QDTORAD * eposPolarity[jointNum - 1]
                      / motorReduction[jointNum - 1]) - qoffset[jointNum - 1];

    // Serial.print("DEBUGING(PLZ REMOVE) READTPDO1 nodeNum =  "); Serial.print(nodeNum);
    // Serial.print(" jointNum =  "); Serial.print(jointNum);
    // Serial.print(" q[jointNum - 1] * RADTODEG =  "); Serial.println(q[jointNum - 1] * RADTODEG);
}

void CANListener(){
  word CANID;
  bool nodesRead[NUMBER_OF_JOINTS];
  bool allNodesRead = false;
  unsigned int tLastSync;

  for(int i = 0; i < NUMBER_OF_JOINTS; i++){ //set all nodes as unread
    nodesRead[i] = false;
  }

  /* TESTING PURPOSES */
  #ifdef TWO_MOTOR_TEST
  for(int i = 2; i < NUMBER_OF_JOINTS; i++){
    nodesRead[i] = true; // not necessary to read any node but first two
    q[i] = q[i] + h * 0.001 * u[i]; // h[ms] assume EPOS respond as perfect integrator
  }
  #endif
  #ifdef SIMU_MODE
  for(int i = 0; i < NUMBER_OF_JOINTS; i++){
    nodesRead[i] = true;
    allNodesRead = true;
    q[i] = q[i] + h * 0.001 * u[i]; // h[ms] assume EPOS respond as perfect integrator
  }
  #endif
  /* END OF TESTING PURPOSES */

  CAN.sendMsgBuf(0x80,0,2,SYNC); // sends Sync object
  tLastSync = millis();

  while(!allNodesRead){
    // read buffer until all nodes have responded to the sync

    if(millis() - tLastSync > PDO_READ_TIMEOUT){
      for(int i = 0; i < NUMBER_OF_JOINTS; i++){
        nodesRead[i] = false;
        }
      Serial.println("WARN: CanListener(): PDO read timeout");
      CAN.sendMsgBuf(0x80,0,2,SYNC); // sends Sync object
      tLastSync = millis();
    }

    if(CAN_MSGAVAIL == CAN.checkReceive()){
      CAN.readMsgBuf(&len, buf);
      CANID = CAN.getCanId();  // read data,  len: data length, buf: data buf

      #ifdef DEBUG_MODE
      Serial.print("DEBUG: CanListener(): 0x");Serial.print(CANID,HEX);Serial.print("\t");
      // print the data
      for(int i = 0; i<len; i++){
        Serial.print("0x");Serial.print(buf[i],HEX);Serial.print("\t");
      }
      Serial.println();
      #endif

      if(CANID == 0x80){
        // Sync object received (we just sent it)
        #ifdef DEBUG_MODE
        Serial.println("DEBUG: CANListener(): Sync object sucessfuly sent");
        #endif
      }
      else if ((CANID > 0x180 + NODEID_OFFSET) && (CANID <= 0x180 + NUMBER_OF_JOINTS + NODEID_OFFSET)){
        // PDO1 received from some node
        nodesRead[CANID - 0x180 - NODEID_OFFSET - 1] = true;
        readTPDO1(CANID);
      }
      else if ((CANID > 0x580 + NODEID_OFFSET) && (CANID <= 0x580 + NUMBER_OF_JOINTS + NODEID_OFFSET ) && (buf[0] == 0x60)){
        // SDO receiving message arrived, confirmation of SDO write object succesfuly received by its intended node
        }
      else if ((CANID > 0x700 + NODEID_OFFSET) && (CANID <= 0x700 + NUMBER_OF_JOINTS + NODEID_OFFSET )){
        // Hearbeat message received from some node
        lastHeartbeat[(CANID - 0x700) - 1] = millis();
      }
      else{
        // other message found, we print it
        Serial.println("WARN: CANListener(): Unexpected message found :");
        Serial.print("WARN: CANListener(): 0x");Serial.print(CAN.getCanId(),HEX);Serial.print("\t");
        for(int i = 0; i<len; i++){
          Serial.print("0x");Serial.print(buf[i],HEX);Serial.print("\t");
        }
        Serial.println();
      }

      allNodesRead = true;
      for(int i = 0; i < NUMBER_OF_JOINTS; i++){
        if(nodesRead[i] == false){
          allNodesRead = false;
        }
      }
    }
  } // while(!allNodesRead)
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
    if (  q[1] > 0  + J_LIMIT_OVP_ALLWD || q[1] < -PI/2 - J_LIMIT_OVP_ALLWD
       || q[2] > PI + J_LIMIT_OVP_ALLWD || q[2] < -PI   - J_LIMIT_OVP_ALLWD
       || q[3] > PI + J_LIMIT_OVP_ALLWD || q[3] < -PI   - J_LIMIT_OVP_ALLWD
     )
    {
        Serial.println("WARN: tetisCheckJointLimits(): Joints execeded limits");
        return true;
    }
    else return false;
}


void checkHearbeat(){
  // checks if hearbeat time has elapsed for any node and, if so, raises an error
  unsigned int numJoints = NUMBER_OF_JOINTS;
  unsigned int nodeNum;
  /* TESTING PURPOSES*/
  #ifdef TWO_MOTOR_TEST
  numJoints = 2;
  #endif
  #ifdef SIMU_MODE
  numJoints = 0;
  #endif
  /* END OF TESTING PURPOSES*/

  for(unsigned int jointNum = 1; jointNum <= numJoints; jointNum++){
    nodeNum = nodeIDMapping[jointNum - 1];
    if(millis() - lastHeartbeat[jointNum - 1] > HEARBEAT_TIME + HB_DELAY_ALWD){
      // a node has not sent hb message last cycle
      Serial.print("WARN: checkHearbeat(): Node "); Serial.print(nodeNum);
      Serial.print(" has not responded in last "); Serial.print(millis() - lastHeartbeat[jointNum - 1] - HEARBEAT_TIME);
      Serial.println(" ms");
    }
    #ifdef DEBUG_MODE
    else{
      Serial.print("DEBUG: checkHearbeat(): Node "); Serial.print(nodeNum);
      Serial.println(" responding");
    }
    #endif
  }
}

void updateControlType(){
  // updates the control type to the one desired by user
  if(initialControl){
    // check user desired mode
    controlType = Joystick;
  }
}

void setup(){
  Serial.begin(USB_BAUDRATE); // init serial comunication (USB->Arduino)
  #if defined(DEBUG_MODE) || defined(TO_MATLAB)
  Serial.println("----------------- RESTART ------------------");
  #endif

  while (CAN_OK != CAN.begin(CAN_BAUDRATE))     // init can bus : baudrate = 500k
  {
      Serial.println("WARN: setup(): CAN BUS Shield init fail");
      Serial.println("WARN: setup(): Init CAN BUS Shield again");
      delay(100);
  }
  #ifdef DEBUG_MODE
  Serial.println("DEBUG: setup(): CAN BUS Shield init ok");
  #endif

  setupShieldJoystick(); // init joystick on the shield

  setupHearbeat();
  toAllNodesSDO(DISABLEVOLTAGE,0); // Send state machine to "Switch On Disable"
  toAllNodesSDO(FAULTRESET,0); // Clear all errors in all nodes (->"Switch On Disable")
  CAN.sendMsgBuf(0x000,0,2,PREOPERATIONAL); printMsgCheck(); // NMT: set CanOpen network to Pre-operational

  setupPDOs();
  setupVelocityMode();
  // setInitialVals();
  // setupHearbeat();

  toAllNodesSDO(SHUTDOWN,0); // Send state machine to "Ready to Switch On"
  toAllNodesSDO(ONANDENABLE,0); // Send state machine to "Operation Enable"
  CAN.sendMsgBuf(0x000,0,2,OPERATIONAL); printMsgCheck(); // NMT: set CanOpen to Operational

  delay(500); //let al the SDOs be attended

  for(int i = 0; i < NUMBER_OF_JOINTS; i++ ){
    qoffset[i] = 0.0 ;
  }
  controlType = Setup;
  // controlType = Joystick;
  tInitPlot = millis(); // zero time for Matlab plot

}


void loop(){
  if((millis() - tLastExec >= h)){
    tDelay = millis() - tLastExec - h;
    if(tDelay > PERMT_DELAY && tLastExec != 0){
      Serial.print("WARN: loop(): Iteration delayed by: "); Serial.print(tDelay);Serial.println(" ms");
    }
    tLastExec = millis();

    // checkHearbeat();

    CANListener(); // get data from EPOS nodes in CAN bus
    updateTetisData(); // update tetis values (cij, cijk,..)

    updateControlType();

    // calculate control
    switch (controlType){
      case Setup:
        #ifdef DEBUG_MODE
        Serial.println("DEBUG: loop(): entering setup");
        #endif
        for(int i = 0; i < NUMBER_OF_JOINTS; i++ ){
          qoffset[i] = q[i] - qinit[i];
          // #ifdef DEBUG_MODE
          Serial.print("DEBUG: loop(): qoffset = ");
          Serial.println(qoffset[i] * RADTODEG);
          // #endif
        }
        controlType =  InitialPosition;
        break;
      case InitialPosition:
        #ifdef DEBUG_MODE
        Serial.println("DEBUG: loop(): entering initial joint control");
        #endif
        initQPosition();
        #ifdef TO_MATLAB
        plotQInMatlab();
        plotUInMatlab();
        #endif
        break;
      case JointControl :
        #ifdef DEBUG_MODE
        Serial.println("DEBUG: loop(): entering joint pos control");
        #endif
        jointPosControl();
        #ifdef TO_MATLAB
        plotQInMatlab();
        plotUInMatlab();
        #endif
        break;
      case Joystick :
        // #ifdef DEBUG_MODE
        Serial.println("DEBUG: loop(): entering joystick control");
        // #endif
        updateDirectKinematics();
        joystickControl();
        #ifdef TO_MATLAB
        plotXInMatlab();
        plotQInMatlab();
        plotUInMatlab();
        #endif
        break;
      case Trajectory :
        #ifdef DEBUG_MODE
        Serial.println("DEBUG: loop(): entering trajectory mode control");
        #endif
        updateDirectKinematics();
        trajectoryControl();
        #ifdef TO_MATLAB
        plotXInMatlab();
        plotQInMatlab();
        plotUInMatlab();
        #endif
        break;
      default :
        Serial.println("WARN: loop(): Type of control invalid or not specifed");
        break;
    }

    uSet(); // send control signal to nodes

  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
