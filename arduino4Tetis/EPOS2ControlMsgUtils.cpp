#include "Arduino4Tetis.h"
#include "EPOS2ControlMsgUtils.h"


byte len = 0;
byte buf[8];
word COBId;

void printMsgCheck(){
// Receive data with check mode
// if send data coming too fast, such as less than 10ms, you can use this way
    if(CAN_MSGAVAIL == CAN.checkReceive()){ // check if data coming
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf
    COBId = CAN.getCanId();
    #ifdef DEBUG_MODE
    Serial.print("DEBUG: printMsgCheck(): 0x");Serial.print(COBId,HEX);Serial.print("\t");
    // print the data
    for(int i = 0; i<len; i++){
      Serial.print("0x");Serial.print(buf[i],HEX);Serial.print("\t");
    }
    Serial.println();
    #endif
  }
}

void toAllNodesSDO(byte* DATA, bool ext){
  //Sends a write SDO message to all nodes in CAN Network
  unsigned int numNodes = NUMBEROFNODES;
  /* TESTING PURPOSES*/
  #ifdef TWO_MOTOR_TEST
  numNodes = 2;
  #endif
  #ifdef SIMU_MODE
  numNodes = 0;
  #endif
  /* END OF TESTING PURPOSES*/
   for(word nodeNum = 1; nodeNum <= numNodes ; nodeNum++){
     CAN.sendMsgBuf(0x600 + nodeNum + NODEID_OFFSET, ext, 8, DATA);
     do{
       // keep printing everything in the buffer until Receving SDO found
       printMsgCheck();
     }while(COBId != 0x580 + nodeNum + NODEID_OFFSET && buf[0] != 60);
   }
}
