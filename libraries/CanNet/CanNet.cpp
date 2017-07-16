// CanNet.cpp


void CanNet :: printBuffer(){
  // Receive data with check mode
  // if send data coming too fast, such as less than 10ms, you can use this way
  byte len = 0;
  byte buf[8];
  word CobId;
  if(CAN_MSGAVAIL == checkReceive()){ // check if data coming
    readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    CobId = getCanId();
    if(user.debugMode){
      serial -> print("CanNet :: printBuffer(): 0x");serial -> print(CobId,HEX);
      // print the data
      for(int i = 0; i<len; i++){
        serial -> print(" 0x");serial -> print(buf[i],HEX);
      }
      serial -> println();
    }
  }
}

void CanNet :: toAllNodesSdo(KinematicSystem* ks,byte* DATA, bool ext){
  //Sends a write SDO message to all nodes of KinematicSystem passed as 1st param
  word nodeId;
  byte len = 0;
  byte buf[8];
  word CobId;
  long unsigned tSend;

  for(int j = 0; j < ks -> numOfJoints; j++){
    nodeId = ks -> systemJoints[j] -> nodeID;
    // if node is not simulated send sdo
    if(! ks -> systemJoints[j] -> simulated ){
      sendMsgBuf(0x600 + nodeId, ext, 8, DATA);
      tSend = millis();
      if(user.debugMode){
        serial -> print("DEBUG:to AllNodesSdo() just sent: 0x");serial -> print(0x600 + nodeId,HEX);
        for(int i = 0; i<7; i++){
          serial -> print(" 0x");serial -> print(DATA[i],HEX);
        }
        serial -> println();
      }
      //wait for nodes sdo response
      do{                                   // print buffer until sdo found
        if(CAN_MSGAVAIL == checkReceive()){ // check if data coming
          readMsgBuf(&len, buf);            // read data,  len: data length, buf: data buf
          CobId = getCanId();
          if(user.debugMode){
            serial -> print("DEBUG: CanNet :: toAllNodesSdo(): 0x");serial -> print(CobId,HEX);
            for(int i = 0; i<len; i++){
              serial -> print(" 0x");serial -> print(buf[i],HEX);
            }
            serial -> println();
          }
        }
        if(millis() - tSend > 50){ // re-send SDO
          Serial.println("WARN: SDO timeout");
          Serial3.println("WARN: SDO timeout");
          sendMsgBuf(0x600 + nodeId, ext, 8, DATA);
          tSend = millis();
        }
      }while(CobId != (0x580 + nodeId));// while(CobId != (0x580 + nodeId))
    }// if(! ks -> systemJoints[j] -> simulated )
  } // for(int j = 0; j < ks -> numOfJoints; j++){
}

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

void CanNet :: uSetPDO(){
  // writes to EPOS the values stored in control variable u (via PDO)
  unsigned int nodeId;
  KinematicSystem* ks;
  Joint* joint;
  long rpm;

  // check inminent collision & joint limits & saturation
  for(int k = 0; k < numOfKinS; k++){
    ks = kinSystems[k];
    #ifndef FORGET_JLMITS_COLIS
      if(ks -> checkCollision() || ks -> checkJointLimits()){
        for(int j = 0; j < ks -> numOfJoints; j++){
          ks -> systemJoints[j] -> u = 0.0;
        }
      }
    #endif // #ifndef FORGET_JLMITS_COLIS
    #ifndef FORGET_SATURATION
      for(int j = 0; j < ks -> numOfJoints; j++){
        joint = ks -> systemJoints[j];

        if (joint->u * RADSTORPM * joint->gearRed > joint-> maxVel){
          // if control > max veloc -> saturate output
          if(user.debugMode){
            serial -> print("DEBUG: uSet(): joint "); serial -> print(j);
            serial -> print(" saturated. u[rad/s] = "); serial -> print(joint->u);
            serial -> print(" u[rpm] = "); serial -> print(joint->u * RADSTORPM);
            serial -> print(" set to[rpm]:   "); serial -> println(joint->maxVel / RADSTORPM / joint->gearRed);
          }
          joint->u = joint->maxVel * RADSTORPM / joint->gearRed;
        }
        else if( joint->u * RADSTORPM * joint->gearRed < - joint-> maxVel){
          // if control > -max veloc -> saturate output
          if(user.debugMode){
            serial -> print("DEBUG: uSet(): joint "); serial -> print(j);
            serial -> print(" saturated. u[rad/s] = "); serial -> print(joint->u);
            serial -> print(" u[rpm] = "); serial -> print(joint->u * RADSTORPM);
            serial -> print(" set to[rpm]:   "); serial -> println( - joint->maxVel / RADSTORPM / joint->gearRed));
            joint -> u = - joint->maxVel * RADSTORPM / joint->gearRed;
          }
      }
    }
    #endif // #ifndef FORGET_SATURATION
  } // for(int k = 0; k < numOfKinS; k++)

  for(int k = 0; k < numOfKinS; k++){
    ks = kinSystems[k];
    serial -> println(ks -> name);
    for(int j = 0; j < ks -> numOfJoints; j++){
      joint = ks -> systemJoints[j];
      if(!(joint -> simulated)){
        nodeId = joint -> nodeID;
        rpm = (joint -> u) * RADSTORPM * (joint -> gearRed) * (joint -> eposPolarity);
        sendMsgBuf(0x200 + nodeId,0,4,(byte*)&rpm);
      }
    }
  }
}

void CanNet :: uSetSDO(){
  // writes to EPOS the values stored in control variable u (via SDO)
  byte WRITE_VELOC[8] = {0x23,0x6B,0x20,0x00,0x00,0x00,0x00,0x00};
  unsigned int nodeId;
  word CobId;
  KinematicSystem* ks;
  byte len;
  byte buf[8];
  Joint* joint;
  long rpm;
  long unsigned tSend;

  // check inminent collision & joint limits & saturation
  for(int k = 0; k < numOfKinS; k++){
    ks = kinSystems[k];
    #ifndef FORGET_JLMITS_COLIS
      if(ks -> checkCollision() || ks -> checkJointLimits()){
        for(int j = 0; j < ks -> numOfJoints; j++){
          ks -> systemJoints[j] -> u = 0.0;
        }
      }
    #endif // #ifndef FORGET_JLMITS_COLIS
    #ifndef FORGET_SATURATION
      for(int j = 0; j < ks -> numOfJoints; j++){
        joint = ks -> systemJoints[j];

        if (joint->u * RADSTORPM * joint->gearRed > joint-> maxVel){
          // if control > max veloc -> saturate output
          if(user.debugMode){
            serial -> print("DEBUG: uSet(): joint "); serial -> print(j);
            serial -> print(" saturated. u[rad/s] = "); serial -> print(joint->u);
            serial -> print(" u[rpm] = "); serial -> print(joint->u * RADSTORPM);
            serial -> print(" set to[rpm]:   "); serial -> println(joint->maxVel / RADSTORPM / joint->gearRed);
          }
          joint->u = joint->maxVel * RADSTORPM / joint->gearRed;
        }
        else if( joint->u * RADSTORPM * joint->gearRed < - joint-> maxVel){
          // if control > -max veloc -> saturate output
          if(user.debugMode){
            serial -> print("DEBUG: uSet(): joint "); serial -> print(j);
            serial -> print(" saturated. u[rad/s] = "); serial -> print(joint->u);
            serial -> print(" u[rpm] = "); serial -> print(joint->u * RADSTORPM);
            serial -> print(" set to[rpm]:   "); serial -> println( - joint->maxVel / RADSTORPM / joint->gearRed));
            joint -> u = - joint->maxVel * RADSTORPM / joint->gearRed;
          }
      }
    }
    #endif // #ifndef FORGET_SATURATION
  } // for(int k = 0; k < numOfKinS; k++)

  for(int k = 0; k < numOfKinS; k++){
    ks = kinSystems[k];
    for(int j = 0; j < ks -> numOfJoints; j++){
      joint = ks -> systemJoints[j];
      if(!(joint -> simulated)){
        nodeId = joint -> nodeID;
        rpm = (joint -> u) * RADSTORPM * (joint -> gearRed) * (joint -> eposPolarity);
        // sendMsgBuf(0x200 + nodeId,0,4,(byte*)&rpm);
        // serial -> print("CanNet :: uSet(): joint "); serial -> print(j+1);
        // serial -> print(" u() = "); serial -> print(joint -> u);
        // serial -> print(" rpm = "); serial -> println(rpm);
        for(int b = 0; b < 4; b++){
          // serial -> print(" buf[i] = "); serial -> print(((byte*)&rpm)[b],HEX);
          WRITE_VELOC[b+4] = ((byte*)&rpm)[b];
        }

        // serial -> println();
        // for(int i = 0; i < 8; i++){
        //   serial -> print(" data[i] = "); serial -> print(WRITE_VELOC[i],HEX);
        // }
        //
        // serial -> println();
        sendMsgBuf(0x600 + nodeId,0,8,WRITE_VELOC);
        tSend = millis();
        do{                                   // print buffer until sdo found
          if(CAN_MSGAVAIL == checkReceive()){ // check if data coming
            readMsgBuf(&len, buf);            // read data,  len: data length, buf: data buf
            CobId = getCanId();
            if(user.debugMode){
              serial -> print("DEBUG: CanNet :: uSet(): 0x");serial -> print(CobId,HEX);
              for(int i = 0; i<len; i++){
                serial -> print(" 0x");serial -> print(buf[i],HEX);
              }
              serial -> println();
            }
          }
          if(millis() - tSend > 20){ // re-send SDO
            Serial.println("WARN: CanNet :: uSet(): SDO timeout");
            Serial3.println("WARN: CanNet :: uSet(): SDO timeout");
            sendMsgBuf(0x600 + nodeId,0,8,WRITE_VELOC);
            tSend = millis();
          }
        }while(CobId != (0x580 + nodeId) || (buf[0] != 0x60));
      }
    }
  }
}

void CanNet :: readPosition(Joint* joint, byte* buf){
    // reads Position from value in qd and bytes given by EPOS
    float qdToRad = 2*PI/(4 * (joint -> encCpr));
    long posInQuadCounts;
    long* auxPointer ;


    auxPointer =  (long*) buf; //access last 4 bytes of TPDO data
    posInQuadCounts = *auxPointer;

    joint -> q = (posInQuadCounts * qdToRad * (joint -> eposPolarity))
                      / (joint -> gearRed) - (joint -> qoffset);
    // serial -> print("DEBUG:CanNet :: readTPDO1(): node "); serial -> print(joint -> nodeID);
    // serial -> print(" : q = "); serial -> println(joint -> q);
    // serial -> print(" : eposPolarity= "); serial -> println(joint -> eposPolarity);
    // serial -> print(" : CPR = "); serial -> println(joint -> encCpr);
    // serial -> print(" : gearRed = "); serial -> println(joint -> gearRed);
    // serial -> print(" : qdToRad = "); serial -> println(qdToRad);
    // serial -> print(" : offset = "); serial -> println(joint -> qoffset);
    // serial -> print(" : posInQuadCounts = "); serial -> println(posInQuadCounts);
}

void CanNet :: canListenerPDO(){
  byte SYNC[2] = {0x00, 0x00};
  word CANID, nodeId;
  bool allNodesRead = false;
  bool msgIdentified;
  byte len = 0;
  byte buf[8];
  Joint* joint;
  long unsigned tLastSync;
  // set all joints to "unsynced" unless they are simulated
  for(int i = 0; i < numOfKinS; i++){
    for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
      joint = kinSystems[i]-> systemJoints[j];
      if(joint -> simulated == true){
        joint -> q +=  h * 0.001 * joint -> u;
        joint -> synced = true;
      }
      else{
        joint -> synced = false;
      }
    }
  }

  sendMsgBuf(0x80,0,2,SYNC); // sends Sync object
  tLastSync = millis();

  serial -> println(millis());
  serial -> println(tLastSync);

  do{// read buffer until all nodes have responded to the sync

    if((millis() - tLastSync) > 20){
      // set all joints to "unsynced" unless they are simulated
      for(int i = 0; i < numOfKinS; i++){
        for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
          joint = kinSystems[i]-> systemJoints[j];
          if(joint -> simulated == false){
            if(user.debugMode){
              serial -> print("Joint "); serial -> print(j); serial -> println(" needs to sync yet");
            }
            joint -> synced = false;
          }
        }
      }
      Serial.println("WARN: canNet::CanListener(): PDO read timeout");
      Serial3.println("WARN: canNet::CanListener(): PDO read timeout");

      sendMsgBuf(0x80,0,2,SYNC); // sends Sync object
      tLastSync = millis();
    } // (millis() - tLastSync) > 20)

    if(CAN_MSGAVAIL == checkReceive()){
      readMsgBuf(&len, buf);
      CANID = getCanId();  // read data,  len: data length, buf: data buf
      msgIdentified = false; // new message

      if(CANID == 0x80){
        // Sync object received (we just sent it)
        if(user.debugMode){
          serial -> println("DEBUG: canNet::CanListener(): SYNC object sent");
        }
      }
      for(int i = 0; i < numOfKinS; i++){
        for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
          // go through all joints to see if identify msg
          joint = kinSystems[i]-> systemJoints[j];
          nodeId = joint -> nodeID;
          if (CANID == (0x180 + nodeId)){
            // PDO1 received from some node
            joint -> synced = true;
            readPosition(joint,buf);
            msgIdentified = true;
          }
          else if(CANID == 0x580 + nodeId){
            // SDO receiving message arrived, confirmation of SDO write object succesfuly received by its intended node
            msgIdentified = true;
          }
          else if(CANID == 0x700 + nodeId){
            // Hearbeat message received from some node
            joint -> tLastHearbeat = millis();
            msgIdentified = true;
          }
        } // for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
      } //for(int i = 0; i < numOfKinS; i++)
      if(!msgIdentified){
        // mesg not identified by any joint, print it
        Serial.println("WARN: CANListener(): Unexpected message found :");
        Serial.println("WARN: CANListener(): Unexpected message found :");
        Serial.print("WARN: CANListener(): 0x");Serial.print(getCanId(),HEX);Serial.print("\t");
        Serial3.print("WARN: CANListener(): 0x");Serial3.print(getCanId(),HEX);Serial3.print("\t");
        for(int i = 0; i<len; i++){
          Serial.print("0x");Serial.print(buf[i],HEX);Serial.print("\t");
          Serial3.print("0x");Serial3.print(buf[i],HEX);Serial3.print("\t");
        }
        Serial.println();
        Serial3.println();
      }
      // check if all joints have been synced
      allNodesRead = true;
      for(int i = 0; i < numOfKinS; i++){
        for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
          if(kinSystems[i]-> systemJoints[j] -> synced == false){
            if(user.debugMode){
              serial -> print("Joint "); serial -> print(j); serial -> println(" not synced");
            }
            allNodesRead = false;
          }
        }
      }
    } // if(CAN_MSGAVAIL == canNet.checkReceive())
  }while(!allNodesRead);
}

void CanNet :: canListenerSDO(){
  byte READ_POSITION[8] = {0x40,0x64,0x60,0x00,0x00,0x00,0x00,0x00};
  word CobId, nodeId;
  long unsigned tSend;
  byte len = 0;
  byte buf[8];
  Joint* joint;
  for(int i = 0; i < numOfKinS; i++){
    for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
      joint = kinSystems[i]-> systemJoints[j];
      nodeId = joint -> nodeID;
      if(! joint -> simulated){
        sendMsgBuf(0x600 + nodeId,0,8,READ_POSITION);
        tSend = millis();
        //wait for nodes sdo response
        // serial -> print("DEBUG: CanNet :: canListenerSDO(): waitting for joint ");
        // serial -> println(j+1);
        do{                                   // print buffer until sdo found
          if(CAN_MSGAVAIL == checkReceive()){ // check if data coming
            readMsgBuf(&len, buf);            // read data,  len: data length, buf: data buf
            CobId = getCanId();
            if(user.debugMode){
              serial -> print("DEBUG: CanNet :: canListenerSDO(): 0x");serial -> print(CobId,HEX);
              for(int i = 0; i<len; i++){
                serial -> print(" 0x");serial -> print(buf[i],HEX);
              }
              serial -> println();
            }
          }
          if(millis() - tSend > 20){ // re-send SDO
            Serial.println("WARN: CanNet :: canListener(): SDO timeout");
            Serial3.println("WARN: CanNet :: canListener(): SDO timeout");
            sendMsgBuf(0x600 + nodeId, 0, 8, READ_POSITION);
            tSend = millis();
          }
        }while(CobId != (0x580 + nodeId) || (buf[0] != 0x43));
        readPosition(joint,(buf+4));
      } // if(! joint -> simulated)
      else{
        (joint -> q) = (joint -> q) + h * 0.001 * (joint -> u);
      }
    }
  }
}

void CanNet :: checkHearbeat(){
  // checks if hearbeat time has elapsed for any node and, if so, raises an error
  KinematicSystem* ks;
  Joint* joint;
  long unsigned delay;
  for(int k = 0; k < numOfKinS; k++){
    ks = kinSystems[k];
    for(int j = 0 ; j < ks -> numOfJoints; j++){
      joint = ks -> systemJoints[j];
      if(millis() - joint -> tLastHearbeat > HEARBEAT_TIME + HB_DELAY_ALWD){
        // a node has not sent hb message last cycle
        delay = millis() - joint -> tLastHearbeat - HEARBEAT_TIME;
        Serial.print("WARN: checkHearbeat(): Node "); Serial.print(joint -> nodeID);
        Serial.print(" has not responded in last "); Serial.print(delay);
        Serial.println(" ms");
        Serial3.print("WARN: checkHearbeat(): Node "); Serial3.print(joint -> nodeID);
        Serial3.print(" has not responded in last "); Serial3.print(delay);
        Serial3.println(" ms");
      }
    }
  }
}

void CanNet :: terminalUtility(){
  byte len = 0;
  byte buf[8];
  word CobId, rCobId;
  byte CanMessage[8];
  char auxString[20];
  bool accessMode; // 0 read, 1 write
  long unsigned int auxInt;
  unsigned int subindex;
  int size;

  while(1){ //subsitutes main loop

    serial -> println("Type 'r' to read an object from an EPOS or 'w' to write");
    while(!serial -> available()){
    }
      switch (serial -> read()){
        case 'r': accessMode = 0; CanMessage[0] = 0x40;  break;
        case 'w':{
          accessMode = 1;
          serial -> println("Type size (number of bytes) of object");
          while(! serial -> available()){
          }
          size = serial -> parseInt();
          switch(size){
            case 1 :
                CanMessage[0] = 0x2F;
                break;
            case 2 :
                CanMessage[0] = 0x2B;
                break;
            case 4 :
                CanMessage[0] = 0x23;
                break;
            default:
                serial -> println("Wrong value, please reboot");
                break;
          }

        } break;
        default: serial -> println("Wrong value, please reboot"); break;
    }

    serial -> println("Type nodeID of node");
    while(!serial -> available()){
      }
    CobId = (serial -> parseInt()) + 0x600;

    serial -> println("Type index of object");
    while(!serial -> available()){
      }
    serial -> readString().toCharArray(auxString,20);
    auxInt = strtoul(auxString, NULL, 16);
    CanMessage[1] = lowByte(auxInt);
    CanMessage[2] = highByte(auxInt);

    serial -> println("Type subindex of object");
    while(! serial -> available()){
      }
    serial -> readString().toCharArray(auxString,20);
    auxInt = strtoul(auxString, NULL, 16);
    CanMessage[3] = byte(auxInt);

    if(accessMode == 0){ // read mode
      CanMessage[4] = CanMessage[5] = CanMessage[6] = CanMessage[7];
    }
    else{ // write mode
      // serial -> println("Type object's size in byte");
      // while(! serial -> available()){
      //   }
      // size = serial -> parseInt();
      for(int i = 0; i < size; i++){
        serial -> print("Type object's byte["); serial -> print(i); serial -> println("]");
        while(! serial -> available()){
          }
        serial -> readString().toCharArray(auxString,20);
        auxInt = strtoul(auxString, NULL, 16);
        CanMessage[i + 4]= byte(auxInt);
      }
      // complete with 0s
      for(int i = size + 4; i < 8; i ++){
        CanMessage[i]= 0;
      }
    }
    canNet.sendMsgBuf(CobId,0,8,CanMessage);

    serial -> println("Debug: just sent this ");
    serial -> print("0x"); serial -> print(CobId,HEX);
    for(int i = 0; i < 8; i++){
      serial -> print(" 0x"); serial -> print(CanMessage[i],HEX);
    }
    serial -> println();

    do{
      if(CAN_MSGAVAIL == canNet.checkReceive()){ // check if data coming
        canNet.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
        rCobId = canNet.getCanId();
          serial -> print("Node responded: 0x");serial -> print(rCobId,HEX);
          // print the data
          for(int i = 0; i<len; i++){
            serial -> print(" 0x");serial -> print(buf[i],HEX);
          }
          serial -> println();
      }

    }while(rCobId != (CobId - 128));
  }//while(1)
}
