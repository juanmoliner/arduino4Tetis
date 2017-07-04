# 1 "/var/folders/2n/jtztfrj56cq6lh_xww26959c0000gn/T/tmp5qPZAl"
#include <Arduino.h>
# 1 "/Users/juanmoliner/Code/arduino4Tetis/arduino4Tetis/Arduino4Tetis.ino"
#include <Arduino.h>

#include <SPI.h>
#include "MatrixMath.h"
#include <math.h>
#include <mcp_can.h>

#include "Arduino4Tetis.h"
#include "EPOS2ControlMsgUtils.h"
#include "EPOS2ControlSetup.h"
#include "ShieldJoystick.h"
#include "TetisKinematics.h"
#include "EPOS2Control.h"
#include "MatlabSerial.h"


long unsigned h = SAMP_TIME;
long unsigned tLastExec;
long unsigned tDelay;

long unsigned lastHeartbeat[NUMBER_OF_JOINTS];

bool initialControl = false;

long unsigned tInitPlot;

unsigned int nodeIDMapping[NUMBER_OF_JOINTS] = NODEID_MAPPING;

char eposPolarity[NUMBER_OF_JOINTS] = EPOS_POLARITY;
unsigned int motorReduction[NUMBER_OF_JOINTS] = MOTOR_REDUCTION;
float maxVelocity[NUMBER_OF_JOINTS] = MAX_VELOCITY;
float maxAcceleration[NUMBER_OF_JOINTS] = MAX_ACCELERATION;


float kp[NUMBER_OF_JOINTS] = KP;
float xd[NUMBER_OF_JOINTS];
float xddot[NUMBER_OF_JOINTS];


float qd[NUMBER_OF_JOINTS];

float x[NUMBER_OF_JOINTS];

float kj[NUMBER_OF_JOINTS] = KJ;
float q[NUMBER_OF_JOINTS];

float u[NUMBER_OF_JOINTS];
float ubar[NUMBER_OF_JOINTS];
float error[NUMBER_OF_JOINTS];

float q0[NUMBER_OF_JOINTS] = Q_INIT_POSITION;
long qEncOffset[NUMBER_OF_JOINTS];
float qinit[NUMBER_OF_JOINTS] = JOINTS_INIT_VALS;

float qoffset[NUMBER_OF_JOINTS];

float c1, s1, c2, s2, c3, s3, c4, s4, c23, s23, c34, s34, c234, s234;
float J0[NUMBER_OF_JOINTS][NUMBER_OF_JOINTS];
float JN[NUMBER_OF_JOINTS][NUMBER_OF_JOINTS];



#ifdef BT_MODE
BluetoothJoystick joystick;
#else
ShieldJoystick joystick;
#endif

byte SYNC[2] = {0x00, 0x00};
byte OPERATIONAL[2] = {0x01, 0};
byte PREOPERATIONAL[2] ={ 0x80, 0};

byte DISABLEVOLTAGE[8] = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
byte SHUTDOWN[8] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
byte FAULTRESET[8] = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
byte ONANDENABLE[8] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};

MCP_CAN CAN(SPI_CS_PIN);



ControlType controlType;
ControlType userControl = InitialPosition;
void uSet();
void readTPDO1(word CANID);
void CANListener();
bool tetisCheckColision();
bool tetisCheckJointLimits();
void checkHearbeat();
void updateControlType();
void setup();
void loop();
#line 86 "/Users/juanmoliner/Code/arduino4Tetis/arduino4Tetis/Arduino4Tetis.ino"
void uSet(){

  unsigned int numJoints = NUMBER_OF_JOINTS;
  unsigned int nodeNum;
  long rpm;

  #ifdef TWO_MOTOR_TEST
  numJoints = 2;
  #endif
  #ifdef SIMU_MODE
  numJoints = 0;
  #endif



  #ifndef FORGET_JLMITS_COLIS
  if(tetisCheckColision() || tetisCheckJointLimits()){

    for(int i = 0; i < NUMBER_OF_JOINTS; i++){
      u[i] = 0.0;
    }
  }
  #endif


  #ifndef FORGET_SATURATION
  for(unsigned int jointNum = 1 ; jointNum <= NUMBER_OF_JOINTS ; jointNum++){
    if (u[jointNum - 1] * RADSTORPM * motorReduction[jointNum - 1] > maxVelocity[jointNum - 1]){


      #ifdef DEBUG_MODE
      Serial.print("DEBUG: uSet(): joint "); Serial.print(jointNum);
      Serial.print(" saturated. u[rad/s] = "); Serial.print(u[jointNum - 1]);
      Serial.print(" u[rpm] = "); Serial.print(u[jointNum - 1] * RADSTORPM);
      Serial.print(" set to[rpm]:   "); Serial.println(maxVelocity[jointNum - 1] / RADSTORPM / motorReduction[jointNum - 1]);
      #endif

      u[jointNum - 1] = maxVelocity[jointNum - 1] * RADSTORPM / motorReduction[jointNum - 1];
    }
    else if( u[jointNum - 1] * RADSTORPM * motorReduction[jointNum - 1] < - maxVelocity[jointNum - 1]){


      #ifdef DEBUG_MODE
      Serial.print("DEBUG: uSet(): joint "); Serial.print(jointNum);
      Serial.print(" saturated. u[rad/s] = "); Serial.print(u[jointNum - 1]);
      Serial.print(" u[rpm] = "); Serial.print(u[jointNum - 1] * RADSTORPM);
      Serial.print(" set to[rpm]:   "); Serial.println( - maxVelocity[jointNum - 1] * RADSTORPM / motorReduction[jointNum - 1]);
      #endif

      u[jointNum - 1] = - (maxVelocity[jointNum - 1] * RADSTORPM / motorReduction[jointNum - 1]);
    }
  }
    #endif

  for(unsigned int jointNum = 1 ; jointNum <= numJoints ; jointNum++){

    nodeNum = nodeIDMapping[jointNum - 1];
    rpm = u[jointNum - 1] * RADSTORPM * motorReduction[jointNum - 1] * eposPolarity[jointNum - 1];
    CAN.sendMsgBuf(0x200 + nodeNum,0,4,(byte*)&rpm);
  }
}


void readTPDO1(word CANID){

    word nodeNum = CANID - 0x180;
    word jointNum;
    long posInQuadCounts;
    long* auxPointer ;


    for(int i = 0; i < NUMBER_OF_JOINTS; i++){
      if(nodeIDMapping[i] == nodeNum){
        jointNum = i + 1;
      }
    }

    auxPointer = (long*) buf;
    posInQuadCounts = *auxPointer;

    q[jointNum - 1] = (posInQuadCounts * QDTORAD * eposPolarity[jointNum - 1]
                      / motorReduction[jointNum - 1]) - qoffset[jointNum - 1];


    Serial.print(" jointNum =  "); Serial.print(jointNum);
    Serial.print(" q[jointNum - 1] * RADTODEG =  "); Serial.println(q[jointNum - 1] * RADTODEG);
}

void CANListener(){
  word CANID;
  bool nodesRead[NUMBER_OF_JOINTS];
  bool allNodesRead = false;
  unsigned int tLastSync;

  for(int i = 0; i < NUMBER_OF_JOINTS; i++){
    nodesRead[i] = false;
  }


  #ifdef TWO_MOTOR_TEST
  for(int i = 2; i < NUMBER_OF_JOINTS; i++){
    nodesRead[i] = true;
    q[i] = q[i] + h * 0.001 * u[i];
  }
  #endif
  #ifdef SIMU_MODE
  for(int i = 0; i < NUMBER_OF_JOINTS; i++){
    nodesRead[i] = true;
    allNodesRead = true;
    q[i] = q[i] + h * 0.001 * u[i];
  }
  #endif


  CAN.sendMsgBuf(0x80,0,2,SYNC);
  tLastSync = millis();

  while(!allNodesRead){


    if(millis() - tLastSync > PDO_READ_TIMEOUT){
      for(int i = 0; i < NUMBER_OF_JOINTS; i++){
        nodesRead[i] = false;
        }
      Serial.println("WARN: CanListener(): PDO read timeout");
      CAN.sendMsgBuf(0x80,0,2,SYNC);
      tLastSync = millis();
    }

    if(CAN_MSGAVAIL == CAN.checkReceive()){
      CAN.readMsgBuf(&len, buf);
      CANID = CAN.getCanId();

      #ifdef DEBUG_MODE
      Serial.print("DEBUG: CanListener(): 0x");Serial.print(CANID,HEX);Serial.print("\t");

      for(int i = 0; i<len; i++){
        Serial.print("0x");Serial.print(buf[i],HEX);Serial.print("\t");
      }
      Serial.println();
      #endif

      if(CANID == 0x80){

        #ifdef DEBUG_MODE
        Serial.println("DEBUG: CANListener(): Sync object sucessfuly sent");
        #endif
      }
      else if ((CANID > 0x180 + NODEID_OFFSET) && (CANID <= 0x180 + NUMBER_OF_JOINTS + NODEID_OFFSET)){

        nodesRead[CANID - 0x180 - NODEID_OFFSET - 1] = true;
        readTPDO1(CANID);
      }
      else if ((CANID > 0x580 + NODEID_OFFSET) && (CANID <= 0x580 + NUMBER_OF_JOINTS + NODEID_OFFSET ) && (buf[0] == 0x60)){

        }
      else if ((CANID > 0x700 + NODEID_OFFSET) && (CANID <= 0x700 + NUMBER_OF_JOINTS + NODEID_OFFSET )){

        lastHeartbeat[(CANID - 0x700) - 1] = millis();
      }
      else{

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
  }
}

bool tetisCheckColision(){



    float p02[3] = {E3*c1*c2, E3*c2*s1, E3*s2};
    float p23[3] = {E4*c1*c2*c3 - E4*c1*s2*s3, E4*c2*c3*s1 - E4*s1*s2*s3, c2*s3 + c3*s2};
    float p03[3];
    float p04[3] = {-M5*s1+E4*c23*c1+E3*c1*c2+E5*c234*c1,
                      M5*c1+E4*c23*s1+E3*c2*s1+E5*c234*s1,
                      E4*s23+E3*s2+E5*s234};
    bool joint3, joint4, eff;
    joint3 = joint4 = eff = false;
    for(int i = 0; i<3; i++){
      p03[i] = p02[i] + p23[i];
    }
    if ( (p02[0] < 200) && (p02[1] < 152 || p02[1] > -152) && (p02[2] > 0)){
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


    if ( ( q[0] > PI/2 || q[0] < -PI/2
          || q[1] > 0 || q[1] < -PI/2
          || q[2] > PI || q[2] < -PI
          || q[3] > PI || q[3] < -PI
        ) && controlType != InitialPosition
     )
    {
        Serial.println("WARN: tetisCheckJointLimits(): Joints execeded limits");
        return true;
    }
    else return false;
}


void checkHearbeat(){

  unsigned int numJoints = NUMBER_OF_JOINTS;
  unsigned int nodeNum;

  #ifdef TWO_MOTOR_TEST
  numJoints = 2;
  #endif
  #ifdef SIMU_MODE
  numJoints = 0;
  #endif


  for(unsigned int jointNum = 1; jointNum <= numJoints; jointNum++){
    nodeNum = nodeIDMapping[jointNum - 1];
    if(millis() - lastHeartbeat[jointNum - 1] > HEARBEAT_TIME + HB_DELAY_ALWD){

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

  if(initialControl){

    controlType = userControl;
  }
}

void setup(){
  Serial.begin(USB_BAUDRATE);
  #ifdef BT_MODE
  Serial3.begin(BT_BAUDRATE);
  #endif
  #if defined(DEBUG_MODE) || defined(TO_MATLAB)
  Serial.println("----------------- RESTART ------------------");
  #endif

  while (CAN_OK != CAN.begin(CAN_BAUDRATE))
  {
      Serial.println("WARN: setup(): CAN BUS Shield init fail");
      Serial.println("WARN: setup(): Init CAN BUS Shield again");
      delay(100);
  }
  #ifdef DEBUG_MODE
  Serial.println("DEBUG: setup(): CAN BUS Shield init ok");
  #endif

  setupShieldJoystick();

  setupHearbeat();
  toAllNodesSDO(DISABLEVOLTAGE,0);
  toAllNodesSDO(FAULTRESET,0);
  CAN.sendMsgBuf(0x000,0,2,PREOPERATIONAL); printMsgCheck();

  setupPDOs();
  setupVelocityMode();



  toAllNodesSDO(SHUTDOWN,0);
  toAllNodesSDO(ONANDENABLE,0);
  CAN.sendMsgBuf(0x000,0,2,OPERATIONAL); printMsgCheck();

  delay(500);

  for(int i = 0; i < NUMBER_OF_JOINTS; i++ ){
    qoffset[i] = 0.0 ;
  }
  controlType = Setup;

  #ifdef TO_MATLAB
  tInitPlot = millis();
  #endif
}


void loop(){
  if((millis() - tLastExec >= h)){
    tDelay = millis() - tLastExec - h;
    if(tDelay > PERMT_DELAY && tLastExec != 0){
      Serial.print("WARN: loop(): Iteration delayed by: "); Serial.print(tDelay);Serial.println(" ms");
    }
    tLastExec = millis();

    #ifdef BT_MODE
    joystick.read();
    #endif


    CANListener();
    updateTetisData();

    updateControlType();


    switch (controlType){
      case Setup:
        #ifdef DEBUG_MODE
        Serial.println("DEBUG: loop(): entering setup");
        #endif
        for(int i = 0; i < NUMBER_OF_JOINTS; i++ ){
          qoffset[i] = q[i] - qinit[i];

          Serial.print("DEBUG: loop(): qoffset = ");
          Serial.println(qoffset[i] * RADTODEG);

        }
        controlType = InitialPosition;
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
      case JoystickBase :

        Serial.println("DEBUG: loop(): entering joystick control");

        updateDirectKinematics();
        joystickBaseControl();
        #ifdef TO_MATLAB
        plotXInMatlab();
        plotQInMatlab();
        plotUInMatlab();
        #endif
        break;
      case JoystickActuator :

        Serial.println("DEBUG: loop(): entering joystick control");

        updateDirectKinematics();
        joystickActuatorControl();
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

    uSet();

  }
}