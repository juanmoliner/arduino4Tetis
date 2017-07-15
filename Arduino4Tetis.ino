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
long unsigned tLastExec = 0; // time(ms)loop was last executed
long unsigned tDelay; // delay from time iteration was supposed to start






HardwareSerial* serial; // ptr to serial port


BluetoothJoystick btJoystick;
ShieldJoystick shJoystick;
Joystick* joystick;


User user(3,false,true,0);


// Joint tetisJoint1(TET_J1_NODEID,TET_J1_RED,TET_J1_VMAX,TET_J1_AMAX,TET_J1_CALQ,TET_J1_CPR,TET_J1_POL,true);
// Joint tetisJoint2(TET_J2_NODEID,TET_J2_RED,TET_J2_VMAX,TET_J2_AMAX,TET_J2_CALQ,TET_J2_CPR,TET_J2_POL,true);
// Joint tetisJoint3(TET_J3_NODEID,TET_J3_RED,TET_J3_VMAX,TET_J3_AMAX,TET_J3_CALQ,TET_J3_CPR,TET_J3_POL,true);
// Joint tetisJoint4(TET_J4_NODEID,TET_J4_RED,TET_J4_VMAX,TET_J4_AMAX,TET_J4_CALQ,TET_J4_CPR,TET_J4_POL,false);
// Joint* jPtr1[4];

Joint tetisJoint1(TET_J1_NODEID,TET_J1_RED,TET_J1_VMAX,TET_J1_AMAX,TET_J1_CALQ,TET_J1_CPR,TET_J1_POL,TETIS_SIMU);
Joint tetisJoint2(TET_J2_NODEID,TET_J2_RED,TET_J2_VMAX,TET_J2_AMAX,TET_J2_CALQ,TET_J2_CPR,TET_J2_POL,TETIS_SIMU);
Joint tetisJoint3(TET_J3_NODEID,TET_J3_RED,TET_J3_VMAX,TET_J3_AMAX,TET_J3_CALQ,TET_J3_CPR,TET_J3_POL,TETIS_SIMU);
Joint tetisJoint4(TET_J4_NODEID,TET_J4_RED,TET_J4_VMAX,TET_J4_AMAX,TET_J4_CALQ,TET_J4_CPR,TET_J4_POL,TETIS_SIMU);
Joint* jPtr1[4];

Joint dorisJoint1(DOR_J1_NODEID,DOR_J1_RED,DOR_J1_VMAX,DOR_J1_AMAX,DOR_J1_CALQ,DOR_J1_CPR,DOR_J1_POL,DORIS_SIMU);
Joint dorisJoint2(DOR_J2_NODEID,DOR_J2_RED,DOR_J2_VMAX,DOR_J2_AMAX,DOR_J2_CALQ,DOR_J2_CPR,DOR_J2_POL,DORIS_SIMU);
Joint dorisJoint3(DOR_J3_NODEID,DOR_J3_RED,DOR_J3_VMAX,DOR_J3_AMAX,DOR_J4_CALQ,DOR_J3_CPR,DOR_J3_POL,DORIS_SIMU);
Joint dorisJoint4(DOR_J4_NODEID,DOR_J4_RED,DOR_J4_VMAX,DOR_J4_AMAX,DOR_J4_CALQ,DOR_J4_CPR,DOR_J4_POL,DORIS_SIMU);
Joint* jPtr2[4];




Tetis tetis(jPtr1);
Doris doris(jPtr2);


KinematicSystem* ksPtr[2];


CanNet canNet(ksPtr,1);

MatlabSerial matlab(MATLAB_PLOT_SAMPLE_T,MATLAB_PREC);




// void CanNet :: uSet(){
//   // writes to EPOS the values stored in control variable u (via PDO)
//   unsigned int nodeId;
//   KinematicSystem* ks;
//   Joint* joint;
//   long rpm;
//
//   // check inminent collision & joint limits & saturation
//   for(int k = 0; k < numOfKinS; k++){
//     ks = kinSystems[k];
//     #ifndef FORGET_JLMITS_COLIS
//       if(ks -> checkCollision() || ks -> checkJointLimits()){
//         for(int j = 0; j < ks -> numOfJoints; j++){
//           ks -> systemJoints[j] -> u = 0.0;
//         }
//       }
//     #endif // #ifndef FORGET_JLMITS_COLIS
//     #ifndef FORGET_SATURATION
//       for(int j = 0; j < ks -> numOfJoints; j++){
//         joint = ks -> systemJoints[j];
//
//         if (joint->u * RADSTORPM * joint->gearRed > joint-> maxVel){
//           // if control > max veloc -> saturate output
//           if(user.debugMode){
//             serial -> print("DEBUG: uSet(): joint "); serial -> print(j);
//             serial -> print(" saturated. u[rad/s] = "); serial -> print(joint->u);
//             serial -> print(" u[rpm] = "); serial -> print(joint->u * RADSTORPM);
//             serial -> print(" set to[rpm]:   "); serial -> println(joint->maxVel / RADSTORPM / joint->gearRed);
//           }
//           joint->u = joint->maxVel * RADSTORPM / joint->gearRed;
//         }
//         else if( joint->u * RADSTORPM * joint->gearRed < - joint-> maxVel){
//           // if control > -max veloc -> saturate output
//           if(user.debugMode){
//             serial -> print("DEBUG: uSet(): joint "); serial -> print(j);
//             serial -> print(" saturated. u[rad/s] = "); serial -> print(joint->u);
//             serial -> print(" u[rpm] = "); serial -> print(joint->u * RADSTORPM);
//             serial -> print(" set to[rpm]:   "); serial -> println( - joint->maxVel / RADSTORPM / joint->gearRed));
//             joint -> u = - joint->maxVel * RADSTORPM / joint->gearRed;
//           }
//       }
//     }
//     #endif // #ifndef FORGET_SATURATION
//   } // for(int k = 0; k < numOfKinS; k++)
//
//   for(int k = 0; k < numOfKinS; k++){
//     ks = kinSystems[k];
//     serial -> println(ks -> name);
//     for(int j = 0; j < ks -> numOfJoints; j++){
//       joint = ks -> systemJoints[j];
//       if(!(joint -> simulated)){
//         nodeId = joint -> nodeID;
//         rpm = (joint -> u) * RADSTORPM * (joint -> gearRed) * (joint -> eposPolarity);
//         sendMsgBuf(0x200 + nodeId,0,4,(byte*)&rpm);
//       }
//     }
//   }
// }

void CanNet :: uSet(){
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






void CanNet :: readTPDO1(Joint* joint, byte* buf){
    // reads TPDO1 and saves it to
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
// void CanNet :: canListener(){
//   byte SYNC[2] = {0x00, 0x00};
//   word CANID, nodeId;
//   bool allNodesRead = false;
//   bool msgIdentified;
//   byte len = 0;
//   byte buf[8];
//   Joint* joint;
//   long unsigned tLastSync;
//   // set all joints to "unsynced" unless they are simulated
//   for(int i = 0; i < numOfKinS; i++){
//     for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
//       joint = kinSystems[i]-> systemJoints[j];
//       if(joint -> simulated == true){
//         joint -> q +=  h * 0.001 * joint -> u;
//         joint -> synced = true;
//       }
//       else{
//         joint -> synced = false;
//       }
//     }
//   }
//
//   sendMsgBuf(0x80,0,2,SYNC); // sends Sync object
//   tLastSync = millis();
//
//   serial -> println(millis());
//   serial -> println(tLastSync);
//
//   do{// read buffer until all nodes have responded to the sync
//
//     if((millis() - tLastSync) > 20){
//       // set all joints to "unsynced" unless they are simulated
//       for(int i = 0; i < numOfKinS; i++){
//         for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
//           joint = kinSystems[i]-> systemJoints[j];
//           if(joint -> simulated == false){
//             if(user.debugMode){
//               serial -> print("Joint "); serial -> print(j); serial -> println(" needs to sync yet");
//             }
//             joint -> synced = false;
//           }
//         }
//       }
//       Serial.println("WARN: canNet::CanListener(): PDO read timeout");
//       Serial3.println("WARN: canNet::CanListener(): PDO read timeout");
//
//       sendMsgBuf(0x80,0,2,SYNC); // sends Sync object
//       tLastSync = millis();
//     } // (millis() - tLastSync) > 20)
//
//     if(CAN_MSGAVAIL == checkReceive()){
//       readMsgBuf(&len, buf);
//       CANID = getCanId();  // read data,  len: data length, buf: data buf
//       msgIdentified = false; // new message
//
//       if(CANID == 0x80){
//         // Sync object received (we just sent it)
//         if(user.debugMode){
//           serial -> println("DEBUG: canNet::CanListener(): SYNC object sent");
//         }
//       }
//       for(int i = 0; i < numOfKinS; i++){
//         for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
//           // go through all joints to see if identify msg
//           joint = kinSystems[i]-> systemJoints[j];
//           nodeId = joint -> nodeID;
//           if (CANID == (0x180 + nodeId)){
//             // PDO1 received from some node
//             joint -> synced = true;
//             readTPDO1(joint,buf);
//             msgIdentified = true;
//           }
//           else if(CANID == 0x580 + nodeId){
//             // SDO receiving message arrived, confirmation of SDO write object succesfuly received by its intended node
//             msgIdentified = true;
//           }
//           else if(CANID == 0x700 + nodeId){
//             // Hearbeat message received from some node
//             joint -> tLastHearbeat = millis();
//             msgIdentified = true;
//           }
//         } // for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
//       } //for(int i = 0; i < numOfKinS; i++)
//       if(!msgIdentified){
//         // mesg not identified by any joint, print it
//         Serial.println("WARN: CANListener(): Unexpected message found :");
//         Serial.println("WARN: CANListener(): Unexpected message found :");
//         Serial.print("WARN: CANListener(): 0x");Serial.print(getCanId(),HEX);Serial.print("\t");
//         Serial3.print("WARN: CANListener(): 0x");Serial3.print(getCanId(),HEX);Serial3.print("\t");
//         for(int i = 0; i<len; i++){
//           Serial.print("0x");Serial.print(buf[i],HEX);Serial.print("\t");
//           Serial3.print("0x");Serial3.print(buf[i],HEX);Serial3.print("\t");
//         }
//         Serial.println();
//         Serial3.println();
//       }
//       // check if all joints have been synced
//       allNodesRead = true;
//       for(int i = 0; i < numOfKinS; i++){
//         for(int j = 0; j < kinSystems[i]-> numOfJoints; j++){
//           if(kinSystems[i]-> systemJoints[j] -> synced == false){
//             if(user.debugMode){
//               serial -> print("Joint "); serial -> print(j); serial -> println(" not synced");
//             }
//             allNodesRead = false;
//           }
//         }
//       }
//     } // if(CAN_MSGAVAIL == canNet.checkReceive())
//   }while(!allNodesRead);
// }

void CanNet :: canListener(){
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
        readTPDO1(joint,(buf+4));
      } // if(! joint -> simulated)
      else{
        (joint -> q) = (joint -> q) + h * 0.001 * (joint -> u);
      }
    }
  }
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

bool Doris :: checkJointLimits(){
  return false;
}

bool Doris :: checkCollision(){
  return false;
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

void terminalUtility(){
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


// void setup(){
//   canNet.begin(CAN_BAUDRATE);
//   Serial.begin(115200);
//   serial = &Serial;
//   terminalUtility();
// }

// void setup2(){
//   long unsigned timer;
//   bool btRead = false;
//
//   byte SYNC[2] = {0x00, 0x00};
//   byte OPERATIONAL[2] = {0x01, 0};
//   byte PREOPERATIONAL[2] ={ 0x80, 0};
//
//   byte DISABLEVOLTAGE[8] = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
//   byte SHUTDOWN[8] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
//   byte FAULTRESET[8] = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
//   byte ONANDENABLE[8] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};
//
//
//   jPtr2[0] = &dorisJoint1;
//   jPtr2[1] = &dorisJoint2;
//   jPtr2[2] = &dorisJoint3;
//   jPtr2[3] = &dorisJoint4;
//
//
//   ksPtr[0] = &doris;
//
//   // ksPtr[0] = &tetis;
//   // ksPtr[1] = &tetis;
//
//   Serial.begin(USB_BAUDRATE);   // start both serials (at least for warnings)
//   Serial3.begin(BT_BAUDRATE);
//
//
//   Serial3.println("Welcome to bt terminal");
//   Serial3.println("Type '9' to start");
//
//   user.useMode = 0;
//   joystick = &btJoystick;
//   joystick -> setup();
//   serial = &Serial;
//
//   joystick -> setup();
//
//   while (CAN_OK != canNet.begin(CAN_BAUDRATE))    // init can bus : baudrate = 500k
//   {
//       Serial.println("WARN: setup(): CAN BUS Shield init fail");
//       Serial3.println("WARN: setup(): CAN BUS Shield init fail");
//       delay(100);
//   }
//   serial -> println("----------------- RESTART ------------------");
//
//   canNet.setupHearbeat(&doris);
//   canNet.toAllNodesSdo(&doris,DISABLEVOLTAGE); // Send state machine to "Switch On Disable"
//   canNet.toAllNodesSdo(&doris,FAULTRESET); // Clear all errors in all nodes (->"Switch On Disable")
//   canNet.sendMsgBuf(0x000,0,2,PREOPERATIONAL); // NMT: set CanOpen network to Pre-operational
//
//   canNet.zeroRPDOs(&doris);
//   canNet.zeroTPDOs(&doris);
//   // canNet.setupRPDO1(&tetis);
//   // canNet.setupTPDO1(&tetis);
//   // canNet.setupVelocityMode(&doris);
//
//   canNet.toAllNodesSdo(&doris,SHUTDOWN); // Send state machine to "Ready to Switch On"
//   canNet.toAllNodesSdo(&doris,ONANDENABLE); // Send state machine to "Operation Enable"
//   canNet.sendMsgBuf(0x000,0,2,OPERATIONAL); // NMT: set CanOpen to Operational
//
//   delay(500); //let al the SDOs be attended
//
//   if(user.debugMode){
//     serial -> println("------------- SETUP DONE -------------");
//   }
// }

void setup(){
  long unsigned timer;
  bool btRead = false;
  char read;

  byte SYNC[2] = {0x00, 0x00};
  byte OPERATIONAL[2] = {0x01, 0};
  byte PREOPERATIONAL[2] ={ 0x80, 0};

  byte DISABLEVOLTAGE[8] = {0x2B, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
  byte SHUTDOWN[8] = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
  byte FAULTRESET[8] = {0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00};
  byte ONANDENABLE[8] = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};

  jPtr1[0] = &tetisJoint1;
  jPtr1[1] = &tetisJoint2;
  jPtr1[2] = &tetisJoint3;
  jPtr1[3] = &tetisJoint4;

  jPtr2[0] = &dorisJoint1;
  jPtr2[1] = &dorisJoint2;
  jPtr2[2] = &dorisJoint3;
  jPtr2[3] = &dorisJoint4;



  ksPtr[0] = &tetis;
  ksPtr[1] = &doris;

  // ksPtr[0] = &doris;
  // ksPtr[1] = &tetis;

  Serial.begin(USB_BAUDRATE);   // start both serials (at least for warnings)
  Serial3.begin(BT_BAUDRATE);

  Serial.println("Waiting for user to specify mode");
  Serial3.println("Welcome to bt terminal");
  Serial3.println("Type 't' to start");

  timer = millis();
  // wait 5 seconds for user to type 9 in terminal or pres START in bt controller
  while(!btRead && (millis() - timer < 5000)){
    btRead = Serial3.available();
  }
  if(btRead){
    read = Serial3.read();
    if(read == 't'){
       user.useMode = 2; // terminal
       joystick = &btJoystick;
       serial = &Serial3;
       serial -> println("BT Terminal mode selected");
    }
    else if(read == 'B'){
      user.useMode = 1; // bt controller
      user.debugMode = false;
      joystick = &btJoystick;
      serial = &Serial3;
    }
  }
  else{
    // nothing was read, asume is PC mode
    user.useMode = 0;
    user.controlType = 4;
    joystick = &shJoystick;
    joystick -> setup();
    serial = &Serial;
    serial -> println("PC mode selected");
  }


  joystick = &btJoystick;
  serial = &Serial3;

  while (CAN_OK != canNet.begin(CAN_BAUDRATE))    // init can bus : baudrate = 500k
  {
      Serial.println("WARN: setup(): CAN BUS Shield init fail");
      Serial3.println("WARN: setup(): CAN BUS Shield init fail");
      delay(100);
  }
  Serial.println("----------------- RESTART ------------------");
  Serial3.println("----------------- RESTART ------------------");

 if(user.useMode == 0 || user.useMode == 1){ // configure PDOs
    canNet.setupHearbeat(&tetis);
    canNet.toAllNodesSdo(&tetis,DISABLEVOLTAGE); // Send state machine to "Switch On Disable"
    canNet.toAllNodesSdo(&tetis,FAULTRESET); // Clear all errors in all nodes (->"Switch On Disable")
    canNet.sendMsgBuf(0x000,0,2,PREOPERATIONAL); // NMT: set CanOpen network to Pre-operational

    canNet.zeroRPDOs(&tetis);
    canNet.zeroTPDOs(&tetis);
    canNet.setupRPDO1(&tetis);
    canNet.setupTPDO1(&tetis);
    canNet.setupVelocityMode(&tetis);

    canNet.toAllNodesSdo(&tetis,SHUTDOWN); // Send state machine to "Ready to Switch On"
    canNet.toAllNodesSdo(&tetis,ONANDENABLE); // Send state machine to "Operation Enable"
    canNet.sendMsgBuf(0x000,0,2,OPERATIONAL); // NMT: set CanOpen to Operational

    delay(500); //let al the SDOs be attended
  }
  else{
    terminalUtility();
  }
  if(user.debugMode){
    serial -> println("------------- SETUP DONE -------------");
  }
}

// void loop1(){
//   // serial -> println("0");
//   if((millis() - tLastExec >= h)){
//     tDelay = millis() - tLastExec - h;
//     if(tDelay > 0 && tLastExec != 0){
//       serial -> print("WARN: loop(): Iteration delayed by: "); serial -> print(tDelay);serial -> println(" ms");
//     }
//     tLastExec = millis();             // reset timer
//     doris.updateControl(1);
//     canNet.uSet();
//   }
// }

void loop(){
  // serial -> println("0");
  if((millis() - tLastExec >= h)){
    tDelay = millis() - tLastExec - h;
    if(tDelay > 0 && tLastExec != 0){
      serial -> print("WARN: loop(): Iteration delayed by: "); serial -> print(tDelay);serial -> println(" ms");
    }
    tLastExec = millis();                       // reset timer
    // serial -> println("1");
    joystick -> read();
    canNet.canListener();                      // get joints (EPOS) data
    // serial -> println("2");
    tetis.updateDirectKinematics();           // q(joint space) -> x(oeprational spc.)
    // serial -> println("3");
    tetis.updateControl(user.controlType);    // calculate control
    // tetis.updateControl(4);    // calculate control
    // serial -> println("4");
    canNet.uSet();                           // send control data to joints
    // serial -> println("4");
  }
}
