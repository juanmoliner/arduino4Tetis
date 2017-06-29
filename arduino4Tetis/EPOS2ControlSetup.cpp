#include "Arduino4Tetis.h"
#include "EPOS2ControlMsgUtils.h"
#include "EPOS2ControlSetup.h"
#include "EPOS2Control.h"
#include "TetisKinematics.h"


void initQPosition(){
  // initial position control in space of the joints to take them out
  // of calibration position
  float q0[NUMBER_OF_JOINTS] = Q_INIT_POSITION;
  float maxError = 0; // max error out of all joints

  for(int i = 0; i < NUMBER_OF_JOINTS; i++){
    qd[i] = q0[i];
  }
  jointPosControl();

  for(int i = 0; i < NUMBER_OF_JOINTS; i++){
    if(abs(error[i]) > maxError) maxError = abs(error[i]);
  }
  #ifdef DEBUG_MODE
  Serial.print("DEBUG: initQPosition(): Max error[deg]: "); Serial.print(maxError * RADTODEG,4);
  Serial.print(" Max error allowed[deg] : "); Serial.println(INIT_Q_MAX_ERROR,4);
  #endif

  if(maxError * RADTODEG < INIT_Q_MAX_ERROR){
    // initial control was fulfilled
    initialControl = true;
    #ifdef DEBUG_MODE
    Serial.println("DEBUG: initQPosition(): Exitting");
    #endif
    for(int i = 0; i < NUMBER_OF_JOINTS; i++){
      u[i] = 0.0; // zero control for safety
    }
  }
}

void setupHearbeat(){
  // configures CAN Hearbeat Protocol
  byte CONS1_HB_TIME[8] ={0x23, 0x16, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00};
  byte CONS2_HB_TIME[8] ={0x23, 0x16, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00};
  byte PROD_HB_TIME[8] = {0x22, 0x17, 0x10, 0x00, lowByte(HEARBEAT_TIME),highByte(HEARBEAT_TIME), 0x00, 0x00};
  unsigned int numNodes = NUMBER_OF_JOINTS;

  /* TESTING PURPOSES*/
  #ifdef TWO_MOTOR_TEST
  numNodes = 2;
  #endif
  #ifdef SIMU_MODE
  numNodes = 0;
  #endif
  /* END OF TESTING PURPOSES*/

  #ifdef DEBUG_MODE
  Serial.println("DEBUG: setupHearbeat(): Setting up hearbeat protocol");
  #endif

  toAllNodesSDO(CONS1_HB_TIME,0);
  toAllNodesSDO(CONS2_HB_TIME,0);
  toAllNodesSDO(PROD_HB_TIME,0);

  // initial time for hearbeat
  for(unsigned int i = 0; i < numNodes; i++){
    lastHeartbeat[i] = millis();
  }
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

  #ifdef DEBUG_MODE
  Serial.println("DEBUG: setupPDOs(): Zeroing al PDOs");
  #endif

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
    Serial.println("DEBUG: setupTPDO1(): Setting up TPDO1");
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

  unsigned long maxProfVelocity[NUMBER_OF_JOINTS] = MAX_VELOCITY;
  unsigned long maxAcceleration[NUMBER_OF_JOINTS] = MAX_ACCELERATION;
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
  // toAllNodesSDO(MAX_PROFVELOC,0);
  // toAllNodesSDO(MAX_ACC,0);
  toAllNodesSDO(ZERO_INITIAL_VELOCITY,0);
}
