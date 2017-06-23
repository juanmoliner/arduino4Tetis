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
  // initial position control in space of the joints to take them out of calibration position

  float maxError = 0; // max error out of all joints
  for(int i = 0; i < NUMBEROFNODES; i++){
    qd[i] = q0[i];
  }
  jointPosControl();

  for(int i = 0; i < NUMBEROFNODES; i++){
    if(abs(error[i]) > maxError) maxError = abs(error[i]);
  }
  #ifdef DEBUG_MODE
  Serial.print("DEBUG: initQPosition(): Max error[deg]: "); Serial.print(maxError * RADTODEG,4);
  Serial.print(" Max error allowed[deg] : "); Serial.println(INIT_Q_MAX_ERROR,4);
  #endif

  if(maxError * RADTODEG < INIT_Q_MAX_ERROR){
    // initial control was fulfilled
    initialControl == true;
    #ifdef DEBUG_MODE
    Serial.println("DEBUG: initQPosition(): Exitting");
    #endif
  }

}




void initQPosition(){
  // Initial Joint control to take them out of singular position
  float q0[NUMBEROFNODES] = Q_INIT_POSITION;
  float maxError; // max error out of all joints
  for(int i = 0; i < NUMBEROFNODES; i++){
    qd[i] = q0[i];
  }

  #ifdef DEBUG_MODE
  Serial.println("DEBUG: initQPosition(): Starting initial joint control");
  #endif

  do{
    if((millis() - tLastExec >= h)){
      tDelay = millis() - tLastExec - h;
      if(tDelay > PERMT_DELAY && tLastExec != 0){
        Serial.print("WARN: initQPosition(): Iteration delayed by: "); Serial.print(tDelay);Serial.println(" ms");
      }
      tLastExec = millis();
      maxError = 0; // resets the maximum error
      CANListener(); // get data from EPOS nodes in CAN bus
      jointPosControl();
      uSet();
      plotQInMatlab();
      plotUInMatlab();
      for(int i = 0; i < NUMBEROFNODES; i++){
        if(abs(error[i]) > maxError) maxError = abs(error[i]);
      }
      #ifdef DEBUG_MODE
      Serial.print("DEBUG: initQPosition(): Max error[deg]: "); Serial.print(maxError * RADTODEG,4);
      Serial.print(" Max error allowed[deg] : "); Serial.println(INIT_Q_MAX_ERROR,4);
      #endif
    }
  }while(maxError * RADTODEG > INIT_Q_MAX_ERROR);
  // leave joints in that position -> zero control
  for(int i = 0; i < NUMBEROFNODES; i++){
    u[i] = 0;
  }
  uSet();
  #ifdef DEBUG_MODE
  Serial.println("DEBUG: initQPosition(): Exitting");
  #endif
}

void setupHearbeat(){
  // configures CAN Hearbeat Protocol
  byte PROD_HB_TIME[8] = {0x22, 0x17, 0x10, 0x00, lowByte(HEARBEAT_TIME),highByte(HEARBEAT_TIME), 0x00, 0x00};

  #ifdef DEBUG_MODE
  Serial.println("DEBUG: setupHearbeat(): Setting up hearbeat protocol");
  #endif

  toAllNodesSDO(PROD_HB_TIME,0);
}





void setupPDOs(){
  // Configures EPOS for TPDO & RPDO transmission
  byte TPDO1_NUM_0[8] = {0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte TPDO2_NUM_0[8] = {0x2F, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte TPDO3_NUM_0[8] = {0x2F, 0x02, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte TPDO4_NUM_0[8] = {0x2F, 0x03, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};

  byte RPDO1_NUM_0[8] = {0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte RPDO2_NUM_0[8] = {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte RPDO3_NUM_0[8] = {0x2F, 0x02, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte RPDO4_NUM_0[8] = {0x2F, 0x03, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};

  // Disable all PDOs in all nodes by writting zero  to number of objects mapped
  toAllNodesSDO(TPDO1_NUM_0,0);
  toAllNodesSDO(TPDO2_NUM_0,0);
  toAllNodesSDO(TPDO3_NUM_0,0);
  toAllNodesSDO(TPDO4_NUM_0,0);

  toAllNodesSDO(RPDO1_NUM_0,0);
  toAllNodesSDO(RPDO2_NUM_0,0);
  toAllNodesSDO(RPDO3_NUM_0,0);
  toAllNodesSDO(RPDO4_NUM_0,0);

  setupTPDO1();

}

void setupTPDO1(){
    // Setup for PDO1 sending Velocity Actual and Position Actual Objects
    byte RESTORE_DEF_PDO_COBID[8] = {0x23, 0x11, 0x10, 0x05, 0x6C, 0x6F, 0x61, 0x64};
    byte TPDO1_INHIBIT_TIME[8] = {0x2B, 0x00, 0x18, 0x03, lowByte(TPDO1_IN_TIME), highByte(TPDO1_IN_TIME), 0x00, 0x00};
    byte TPDO1_TRANSMTYPE[8] = {0x2F, 0x00, 0x18, 0x02, TPDO1_TR_TYPE, 0x00, 0x00, 0x00};
    byte TPDO1_NUM_0[8] = {0x2F,0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte TPDO1_NUM_2[8] = {0x2F,0x00, 0x1A, 0x00, 0x02, 0x00, 0x00, 0x00};
    byte TPDO1_1MPO_VELOCITYACTUAL[8] = {0x23, 0x00, 0x1A, 0x01, 0x20, 0x00, 0x6C, 0x60};
    byte TPDO1_2MPO_POSITIONACTUAL[8] = {0x23, 0x00, 0x1A, 0x02, 0x20, 0x00, 0x64, 0x60};

    #ifdef DEBUG_MODE
    Serial.println("DEBUG: setupTPDOs(): Setting up TPDO1");
    #endif

    // recalculate al COB-ID of the PDO's based on their DIP-Switch
    toAllNodesSDO(RESTORE_DEF_PDO_COBID,0);
    // set TPDO1 inhibit time
    toAllNodesSDO(TPDO1_INHIBIT_TIME,0);
    // set TPDO1 transmission type
    toAllNodesSDO(TPDO1_TRANSMTYPE,0);
    // Set number of mapped objects to 0 to be able to change mapping
    toAllNodesSDO(TPDO1_NUM_0,0);
    // set mapped objects and number of mapped objects
    toAllNodesSDO(TPDO1_1MPO_VELOCITYACTUAL,0);
    toAllNodesSDO(TPDO1_2MPO_POSITIONACTUAL,0);
    toAllNodesSDO(TPDO1_NUM_2,0);
}



void setupVelocityMode(){
  byte MAX_PROFVELOC[8] = {0x23, 0x7F, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte MAX_ACC[8] = {0x23, 0xC5, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte MODE_VELOCITY[8] = {0x2F, 0x60, 0x60, 0x00, 0xFE, 0x00, 0x00, 0x00};
  byte ZERO_INITIAL_VELOCITY[8] = {0x23, 0x6B, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};

  unsigned long maxProfVelocity[NUMBEROFNODES] = MAX_VELOCITY;
  unsigned long maxAcceleration[NUMBEROFNODES] = MAX_ACCELERATION;
  byte* maxProfVelocBytes;
  byte* maxAcceleratBytes;

  #ifdef DEBUG_MODE
  Serial.println("DEBUG: setupVelocityMode(): Setting up velocity mode");
  #endif

  // to be able to acces value byte by byte
  for(int i = 0; i < 4; i++){
    maxProfVelocBytes = (byte*) (maxProfVelocity + i);
    maxAcceleratBytes = (byte*) (maxAcceleration + i);
    MAX_PROFVELOC[i + 4] = maxProfVelocBytes[i];
    MAX_ACC[i + 4] = maxAcceleratBytes[i];
  }

  toAllNodesSDO(MODE_VELOCITY,0);
  toAllNodesSDO(MAX_PROFVELOC,0);
  toAllNodesSDO(MAX_ACC,0);
  toAllNodesSDO(ZERO_INITIAL_VELOCITY,0);
}



void setInitialVals(){
  unsigned char nodesRemaining = NUMBEROFNODES;
  word CANID, nodeNum;
  long* auxPointer;

  /* TESTING PURPOSES */
  #ifdef TWO_MOTOR_TEST
  nodesRemaining = 2;
  #endif
  #ifdef SIMU_MODE
  nodesRemaining = 0;
  #endif
  /* END OF TESTING PURPOSES */

  #ifdef DEBUG_MODE
  Serial.println("DEBUG: setInitialVals(): Setting initial values");
  #endif

  CAN.sendMsgBuf(0x80,0,2,SYNC); // sends Sync object
  while(nodesRemaining > 0){
    // read buffer until all nodes have responded to the sync
    CAN.readMsgBuf(&len, buf);
    CANID = CAN.getCanId();  // read data,  len: data length, buf: data buf
    if(CANID == 0x80){
      #ifdef DEBUG_MODE
      Serial.println("DEBUG: setInitialVals(): Sync object sucessfuly sent");
      #endif
    }
    else if( CANID > 0x180 + NODEID_OFFSET && CANID <= 0x180 + NUMBEROFNODES + NODEID_OFFSET){
      // read TPDO1;

      nodeNum = CANID - 0x180;
      auxPointer =  (long*) (buf + 4); //access 4 last bytes of TPDO data
      encQdOffset[nodeNum - 1] = *auxPointer;

      u[nodeNum - 1] = 0.0; // Set init control variable to 0 for safety

      #ifdef DEBUG_MODE
      Serial.print("DEBUG: setInitialVals(): Initial encoder counts[qd] node ");
      Serial.print(nodeNum);
      Serial.print(": encQdOffset = ");
      Serial.println(encQdOffset[nodeNum - 1]);
      #endif

      nodesRemaining--;
    }
    else{
      // other message found, we print it
      #ifdef DEBUG_MODE
      Serial.println("DEBUG: setInitialVals(): Message different from TPDO1 found:");
      Serial.print("DEBUG: setInitialVals(): 0x");Serial.print(CAN.getCanId(),HEX);Serial.print("\t");
      for(int i = 0; i<len; i++){
        Serial.print("0x");Serial.print(buf[i],HEX);Serial.print("\t");
      }
      Serial.println();
      #endif
    }
  } // while(nodesRemaining > 0)
}



// void setInitialVals(){
//   //sets initial values of position(incremental encoder)
//   byte READINCENCOD1[8] = {0x40, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
//   long unsigned* auxPointer;
//   unsigned int numNodes = NUMBEROFNODES;
//
//   #ifdef TWO_MOTOR_TEST
//   numNodes = 2;
//   u[2] = u[3] = 0.0;
//   #endif
//   #ifdef SIMU_MODE
//   numNodes = 0;
//   u[0] = u[1] = u[2] = u[3] = 0.0;
//   #endif
//
//   for(word nodeNum = 1; nodeNum <= numNodes; nodeNum++){
//     #ifdef DEBUG_MODE
//     Serial.print("DEBUG: setInitialVals(): Setting encoder initial counts for node ");
//     Serial.println(nodeNum);
//     #endif
//     CAN.sendMsgBuf(0x600 + nodeNum + NODEID_OFFSET,0,8,READINCENCOD1);
//     delay(10);
//     do{
//       // keep printing everything in the buffer until Receiving SDO found
//       printMsgCheck();
//     }while(COBId != 0x580 + nodeNum + NODEID_OFFSET && buf[0] != 0x43
//            && buf[1] != 0x20 && buf[2] != 0x20 );
//
//     auxPointer = (long unsigned*) (buf + 4);
//     encQdOffset[nodeNum - 1] = *auxPointer;
//
//     #ifdef DEBUG_MODE
//     Serial.print("DEBUG: setInitialVals(): Initial counts[qd] set: ");
//     Serial.println(encQdOffset[nodeNum - 1]);
//     #endif
//
//     u[nodeNum - 1] = 0.0; // Set init control variable to 0 for safety
//   }
//
// }
