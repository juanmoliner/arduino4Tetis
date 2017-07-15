#include "Arduino4Tetis.h"
#include "EPOS2ControlMsgUtils.h"
#include "EPOS2ControlSetup.h"
#include "EPOS2Control.h"
#include "TetisKinematics.h"


void CanNet :: setupHearbeat(KinematicSystem* ks, unsigned int hbTime){
  // configures CAN Hearbeat Protocol
  byte CONS1_HB_TIME[8] ={0x23, 0x16, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00};
  byte CONS2_HB_TIME[8] ={0x23, 0x16, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00};
  byte PROD_HB_TIME[8] = {0x22, 0x17, 0x10, 0x00, lowByte(hbTime),highByte(hbTime), 0x00, 0x00};


  if(user.debugMode){
    serial -> println("DEBUG: CanNet::setupHearbeat(): Setting up hearbeat protocol");
  }


  toAllNodesSdo(ks,CONS1_HB_TIME);
  toAllNodesSdo(ks,CONS2_HB_TIME);
  toAllNodesSdo(ks,PROD_HB_TIME);


  for(int j; j< ks -> numOfJoints; j++){
    ks -> systemJoints[j] -> tLastHearbeat = millis();
  }
}
void CanNet :: zeroTPDOs(KinematicSystem* ks){
  // Zeros all TPDOs configured in ks epos
  byte TPDO1_NUM_0[8] = {0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte TPDO2_NUM_0[8] = {0x2F, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte TPDO3_NUM_0[8] = {0x2F, 0x02, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte TPDO4_NUM_0[8] = {0x2F, 0x03, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};

  if(user.debugMode){
    serial -> println("DEBUG: CanNet :: zeroTPDOs(): zeroing all TPDOS");
  }

  // Disable all PDOs in all nodes by writting zero  to number of objects mapped
  toAllNodesSdo(ks,TPDO1_NUM_0);
  toAllNodesSdo(ks,TPDO2_NUM_0);
  toAllNodesSdo(ks,TPDO3_NUM_0);
  toAllNodesSdo(ks,TPDO4_NUM_0);
}

void CanNet :: zeroRPDOs(KinematicSystem* ks){
  // Zeros all RPDOs configured in ks epos
  byte RPDO1_NUM_0[8] = {0x2F, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte RPDO2_NUM_0[8] = {0x2F, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte RPDO3_NUM_0[8] = {0x2F, 0x02, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte RPDO4_NUM_0[8] = {0x2F, 0x03, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};

  if(user.debugMode){
    serial -> println("DEBUG: CanNet :: zeroRPDOs(): zeroing all RPDOS");
   }

  // Disable all PDOs in all nodes by writting zero  to number of objects mapped
  toAllNodesSdo(ks,RPDO1_NUM_0);
  toAllNodesSdo(ks,RPDO2_NUM_0);
  toAllNodesSdo(ks,RPDO3_NUM_0);
  toAllNodesSdo(ks,RPDO4_NUM_0);
}

void CanNet :: setupRPDO1(KinematicSystem* ks){
    // Setup for RPDO1 receiving Velocity Actual
    byte RPDO1_INHIBIT_TIME[8] = {0x2B, 0x00, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00};
    byte RPDO1_TRANSMTYPE[8] = {0x2F, 0x00, 0x14, 0x02, 255, 0x00, 0x00, 0x00};
    byte RPDO1_NUM_0[8] = {0x2F,0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte RPDO1_NUM_1[8] = {0x2F,0x00, 0x16, 0x00, 0x01, 0x00, 0x00, 0x00};
    byte RPDO1_1MPO_VEL_MOD_SET_VAL[8] = {0x23, 0x00, 0x16, 0x01, 0x20, 0x00, 0x6B, 0x20};

    if(user.debugMode){
      serial -> println("DEBUG: setupRPDO1(): Setting up RPDO1");
    }
    // set TPDO1 inhibit time
    toAllNodesSdo(ks,RPDO1_INHIBIT_TIME);
    // set TPDO1 transmission type
    toAllNodesSdo(ks,RPDO1_TRANSMTYPE);
    // Set number of mapped objects to 0 to be able to change mapping
    toAllNodesSdo(ks,RPDO1_NUM_0);
    // set mapped objects and number of mapped objects
    toAllNodesSdo(ks,RPDO1_1MPO_VEL_MOD_SET_VAL);
    toAllNodesSdo(ks,RPDO1_NUM_1);
}

void CanNet :: setupTPDO1(KinematicSystem* ks){
    // Setup for PDO1 sending Velocity Actual and Position Actual Objects
    byte TPDO1_INHIBIT_TIME[8] = {0x2B, 0x00, 0x18, 0x03, 0x00, 0x00, 0x00, 0x00};
    byte TPDO1_TRANSMTYPE[8] = {0x2F, 0x00, 0x18, 0x02, 1, 0x00, 0x00, 0x00};
    byte TPDO1_NUM_0[8] = {0x2F,0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte TPDO1_NUM_1[8] = {0x2F,0x00, 0x1A, 0x00, 0x01, 0x00, 0x00, 0x00};
    byte TPDO1_1MPO_POSITIONACTUAL[8] = {0x23, 0x00, 0x1A, 0x01, 0x20, 0x00, 0x64, 0x60};

    if(user.debugMode){
      serial -> println("DEBUG: setupTPDO1(): Setting up TPDO1");
    }

    // set TPDO1 inhibit time
    toAllNodesSdo(ks,TPDO1_INHIBIT_TIME);
    // set TPDO1 transmission type
    toAllNodesSdo(ks,TPDO1_TRANSMTYPE);
    // Set number of mapped objects to 0 to be able to change mapping
    toAllNodesSdo(ks,TPDO1_NUM_0);
    // set mapped objects and number of mapped objects
    toAllNodesSdo(ks,TPDO1_1MPO_POSITIONACTUAL);
    toAllNodesSdo(ks,TPDO1_NUM_1);
}





void CanNet :: setupVelocityMode(KinematicSystem* ks){
  byte MAX_PROFVELOC[8] = {0x23, 0x7F, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte MAX_ACC[8] = {0x23, 0xC5, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte MODE_VELOCITY[8] = {0x2F, 0x60, 0x60, 0x00, 0xFE, 0x00, 0x00, 0x00};
  byte ZERO_INITIAL_VELOCITY[8] = {0x23, 0x6B, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};

  Joint* joint;
  unsigned long maxProfVelocity;
  unsigned long maxAcceleration;
  byte* maxProfVelocBytes;
  byte* maxAcceleratBytes;

  if(user.debugMode){
    serial -> println("DEBUG: CanNet :: setupVelocityMode(): stting up v mode");
  }

  for(int j = 0; j < ks -> numOfJoints; j++){
    joint = ks -> systemJoints[j];
    maxProfVelocity = joint -> maxVel;
    maxAcceleration = joint -> maxAcc;
    // to be able to acces value byte by byte
    for(int i = 0; i < 4; i++){
      maxProfVelocBytes = (byte*) &maxProfVelocity;
      maxAcceleratBytes = (byte*) &maxAcceleration;
      MAX_PROFVELOC[i + 4] = maxProfVelocBytes[i];
      MAX_ACC[i + 4] = maxAcceleratBytes[i];
    }
    sendMsgBuf(0x600 + (joint -> nodeID),0,8,MODE_VELOCITY);
    // sendMsgBuf(0x600 + (joint -> nodeID),0,8,MAX_PROFVELOC);
    // sendMsgBuf(0x600 + (joint -> nodeID),0,8,MAX_ACC);
    sendMsgBuf(0x600 + (joint -> nodeID),0,8,ZERO_INITIAL_VELOCITY);
  }
}
