#include "Arduino4Tetis.h"
#include "EPOS2ControlMsgUtils.h"
#include "EPOS2ControlSetup.h"
#include "EPOS2Control.h"
#include "TetisKinematics.h"


void initXPosition(){
  // Takes actuator to initial x position(mm)(space of the actuator)
  // Control: Prop+FF until max error < INIT_X_MAX_ERROR
  float x0[NUMBEROFNODES] = X_INIT_POSITION;
  float maxError; // max error out of all coordenates
  for(int i = 0; i < NUMBEROFNODES; i++){
    xd_h[i] = x0[i];
  }
  do{
    maxError = 0; // resets the maximum error
    CANListener(); // get data from EPOS nodes in CAN bus
    updateTetisData(); // update tetis values (cij, cijk,..)
    updateDirectKinematics();
    proportionalFF();
    uSet();
    for(int i = 0; i < NUMBEROFNODES; i++){
      if(abs(error[i]) > maxError) maxError = abs(error[i]);
    }
    #ifdef DEBUG_MODE
    Serial.print("DEBUG: initXPosition(): Max error: "); Serial.println(maxError);
    #endif
  }while(maxError > INIT_X_MAX_ERROR);
  // leave actuator in that position -> zero control
  for(int i = 0; i < NUMBEROFNODES; i++){
    u[i] = 0.0;
  }
  uSet();
  #ifdef DEBUG_MODE
  Serial.println("DEBUG: initXPosition(): exitting");
  #endif
}




void initQPosition(){
  // Initial Joint control to take them out of singular position
  float q0[NUMBEROFNODES] = Q_INIT_POSITION;
  float maxError; // max error out of all joints
  for(int i = 0; i < NUMBEROFNODES; i++){
    qd[i] = q0[i];
  }
  do{
    maxError = 0; // resets the maximum error
    CANListener(); // get data from EPOS nodes in CAN bus
    jointPosControl();
    uSet();
    for(int i = 0; i < NUMBEROFNODES; i++){
      if(abs(error[i]) > maxError) maxError = abs(error[i]);
    }
    #ifdef DEBUG_MODE
    Serial.print("DEBUG: initQPosition(): Max error: "); Serial.println(maxError);
    #endif
  }while(maxError > INIT_Q_MAX_ERROR);
  // leave joints in that position -> zero control
  for(int i = 0; i < NUMBEROFNODES; i++){
    u[i] = 0;
  }
  uSet();
  #ifdef DEBUG_MODE
  Serial.println("DEBUG: initQPosition(): Exitting");
  #endif
}

void setupTPDOs(){
    // Configures the EPOS to send the desired objects via PDO
    byte RESTORE_DEF_PDO_COBID[8] = {0x23, 0x11, 0x10, 0x05, 0x64, 0x61, 0x6F, 0x6C};
    byte TPDO1_INHIBIT_TIME[8] = {0x2B, 0x00, 0x18, 0x03, lowByte(TPDO1_IN_TIME), highByte(TPDO1_IN_TIME), 0x00, 0x00};
    byte TPDO1_TRANSMTYPE[8] = {0x2F, 0x00, 0x18, 0x02, TPDO1_TR_TYPE, 0x00, 0x00, 0x00};
    byte TPDO1_NUM_0[8] = {0x2F,0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
    // byte TPDO1_NUM_1[8] = {0x2F,0x00, 0x1A, 0x00, 0x01, 0x00, 0x00, 0x00};
    byte TPDO1_NUM_2[8] = {0x2F,0x00, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00};
    // byte TPDO1_1MPO_STATUSWORD[8] = {0x23, 0x00, 0x1A, 0x01, 0x10, 0x00, 0x41, 0x60};
    byte TPDO1_1MPO_VELOCITYACTUAL[8] = {0x23, 0x00, 0x1A, 0x01, 0x20, 0x00, 0x6C, 0x60};
    byte TPDO1_2MPO_INC_ENCOD1_COUNT[8] = {0x23, 0x00, 0x1A, 0x02, 0x20, 0x00, 0x20, 0x20};
    // byte TPDO1_2MPO_VELOCITYACTUAL[8] = {0x23, 0x00, 0x1A, 0x02, 0x20, 0x00, 0x6C, 0x60};

    // recalculate al COB-ID of the PDO's based on their DIP-Switch
    toAllNodesSDO(RESTORE_DEF_PDO_COBID,0);
    // set TPDO1 inhibit time
    toAllNodesSDO(TPDO1_INHIBIT_TIME,0);
    // set TPDO1 transmission type
    toAllNodesSDO(TPDO1_TRANSMTYPE,0);
    // Set number of mapped objects to 0 to be able to change mapping
    toAllNodesSDO(TPDO1_NUM_0,0);
    // set mapped objects and number of mapped objects
    // toAllNodesSDO(TPDO1_1MPO_STATUSWORD,0);
    toAllNodesSDO(TPDO1_1MPO_VELOCITYACTUAL,0);
    toAllNodesSDO(TPDO1_2MPO_INC_ENCOD1_COUNT,0);
    toAllNodesSDO(TPDO1_NUM_2,0);
}

void setupVelocityMode(){
  byte MAX_PROFVELOC[8] = {0x23, 0x7F, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte MAX_ACC[8] = {0x23, 0xC5, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte MODE_VELOCITY[8] = {0x2F, 0x60, 0x60, 0x00, 0xFE, 0x00, 0x00, 0x00};
  byte ZERO_INITIAL_VELOCITY[8] = {0x23, 0x6B, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};

  unsigned long maxProfVelocity = 6250;
  unsigned long maxAcceleration = 6250;
  // to be able to acces value byte by byte
  byte* maxProfVelocBytes = (byte*) &maxProfVelocity;
  byte* maxAcceleratBytes = (byte*) &maxAcceleration;
  for(int i = 0; i < 4; i++){
    MAX_PROFVELOC[i + 4] = maxProfVelocBytes[i];
    MAX_ACC[i + 4] = maxAcceleratBytes[i];
    }

  toAllNodesSDO(MODE_VELOCITY,0);
  toAllNodesSDO(MAX_PROFVELOC,0);
  toAllNodesSDO(MAX_ACC,0);
  toAllNodesSDO(ZERO_INITIAL_VELOCITY,0);
}


void setInitialVals(){
  //sets initial values of position(incremental encoder)
  byte READINCENCOD1[8] = {0x40, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
  long unsigned posInQuadCounts;
  long unsigned* auxPointer;
  unsigned int numNodes = NUMBEROFNODES;

  #ifdef TWO_MOTOR_TEST
  numNodes = 2;
  u[2] = u[3] = 0.0;
  #endif
  #ifdef SIMU_MODE
  numNodes = 0;
  u[0] = u[1] = u[2] = u[3] = 0.0;
  #endif

  for(word nodeNum = 1; nodeNum <= numNodes; nodeNum++){
    CAN.sendMsgBuf(0x600 + nodeNum + NODEID_OFFSET,0,8,READINCENCOD1);
    delay(10);
    do{
      // keep printing everything in the buffer until Receiving SDO found
      printMsgCheck();
    }while(COBId != 0x580 + nodeNum + NODEID_OFFSET && buf[0] != 43);
    auxPointer = (long unsigned*) (buf + 4);
    posInQuadCounts = *auxPointer;
    // /* DEBUGGING PURPOSES */
    // #ifdef DEBUG_MODE
    // Serial.print("Quadcounts iniciales del nodo "); Serial.print(nodeNum);
    // Serial.print(" : "); Serial.println(posInQuadCounts);
    // #endif
    // /* END OF DEBUGGING PURPOSES */
    qoffset[nodeNum - 1] = (float)posInQuadCounts * QDTORAD / motorReduction[nodeNum - 1];
    u[nodeNum - 1] = 0.0; // Set init control variable to 0 for safety
  }

}
