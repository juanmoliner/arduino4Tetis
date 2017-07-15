#include "Arduino4Tetis.h"
#include "EPOS2ControlMsgUtils.h"


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
