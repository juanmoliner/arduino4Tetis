#include <Arduino.h>

#include "CanNet.h"
#include "KinematicSystem.h"
#include "Doris.h"
#include "Tetis.h"
#include "MatlabSerial.h"
#include "TetisJoystick.h"



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
    canNet.terminalUtility();
  }
  if(user.debugMode){
    serial -> println("------------- SETUP DONE -------------");
  }
}

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
    canNet.canListenerSDO();                      // get joints (EPOS) data
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
