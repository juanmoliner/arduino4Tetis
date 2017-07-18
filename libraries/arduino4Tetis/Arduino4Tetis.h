#ifndef ARDUINO4TETIS_H
#define ARDUINO4TETIS_H

class Joint;
class KinematicSystem;
class User;
class CanNet;

#include <mcp_can.h>
#include <MatrixMath.h>
#include "CanNet.h"
#include "KinematicSystem.h"
#include "Doris.h"
#include "Tetis.h"
#include "MatlabSerial.h"
#include "TetisJoystick.h"

/* HELPFUL UNIT CONVERSION */
#define RADSTORPM (60/(2*PI)) // rpm /(rad/s)
#define RPMTORADS ((2*PI)/60)
#define RADTODEG (180/PI) // grad/rad
#define DEGTORAD (PI/180)

/* DORIS DATA */
// if 1 all Doris Joints are simulated
#define DORIS_SIMU true
// Joint's node id
#define DOR_J1_NODEID 1
#define DOR_J2_NODEID 2
#define DOR_J3_NODEID 3
#define DOR_J4_NODEID 4
// Joints gear reduction (i.e 100 = "1:100")
#define DOR_J1_RED 21
#define DOR_J2_RED 21
#define DOR_J3_RED 21
#define DOR_J4_RED 21
// Joints max velocity [rpm @ motor]
#define DOR_J1_VMAX 6000
#define DOR_J2_VMAX 6000
#define DOR_J3_VMAX 6000
#define DOR_J4_VMAX 6000
// Joints max acceleration [rpm/s @ motor]
#define DOR_J1_AMAX 600
#define DOR_J2_AMAX 600
#define DOR_J3_AMAX 600
#define DOR_J4_AMAX 600
// Joints calibration (initial) angle [rad]
#define DOR_J1_CALQ 0
#define DOR_J2_CALQ 0
#define DOR_J3_CALQ 0
#define DOR_J4_CALQ 0
// Joints encoder counts per revolution
#define DOR_J1_CPR 500
#define DOR_J2_CPR 500
#define DOR_J3_CPR 500
#define DOR_J4_CPR 500
// Epos polarity (1: hourly angles positive)
#define DOR_J1_POL 1
#define DOR_J2_POL -1
#define DOR_J3_POL 1
#define DOR_J4_POL -1

/* TETIS DATA */
// if 1 all Tetis Joints are simulated
#define TETIS_SIMU false
// Joint's node id
#define TET_J1_NODEID 9
#define TET_J2_NODEID 8
#define TET_J3_NODEID 7
#define TET_J4_NODEID 6
// Joints gear reduction (i.e 100 = "1:100")
#define TET_J1_RED 100
#define TET_J2_RED 100
#define TET_J3_RED 100
#define TET_J4_RED 100
// Joints max velocity [rpm @ motor]
#define TET_J1_VMAX 650
#define TET_J2_VMAX 650
#define TET_J3_VMAX 650
#define TET_J4_VMAX 650
// Joints max acceleration [rpm/s @ motor]
#define TET_J1_AMAX 200
#define TET_J2_AMAX 200
#define TET_J3_AMAX 200
#define TET_J4_AMAX 200
// Joints calibration (initial) angle [rad]
#define TET_J1_CALQ 0
#define TET_J2_CALQ -PI/2
#define TET_J3_CALQ 0
#define TET_J4_CALQ 0
// Joints encoder counts per revolution
#define TET_J1_CPR 2000
#define TET_J2_CPR 2000
#define TET_J3_CPR 2000
#define TET_J4_CPR 2000
// Epos polarity (1: hourly angles positive)
#define TET_J1_POL -1
#define TET_J2_POL 1
#define TET_J3_POL -1
#define TET_J4_POL -1


/* BLUETOOTH */
// #define BT_MODE // wether bluetooth is used
#define BT_BAUDRATE 57600

/* MATLAB PLOTTING MODE*/
#define TO_MATLAB // if defined sends data to be plotted to Matlab
#define MATLAB_PREC 3 // number of decimals to send to Matlab
#define MATLAB_PLOT_SAMPLE_T 60 //Matlab plotting sample time(ms)

/*  TEST MODE */
#define SIMU_MODE // simulation mode (overrides TWO_MOTOR_TEST)
// #define TWO_MOTOR_TEST // 2 motors simulated + 2 real

/* DEBUG MODE */
#define DEBUG_PREC 3 // number of decimals to show in debug mode

/* CONTROL PARAMETERS*/
#define FORGET_JLMITS_COLIS // define to take of joint and collision limits
#define FORGET_SATURATION // define to take off saturation limits
#define SAMP_TIME 40 // Sampling time (ms) for the control loops
#define K_JOYSTICK  50 // joystick gain
#define INIT_Q_MAX_ERROR 0.3 // max error(qd-q)[grad] allowed in initial joint control out of singular
#define Q_INIT_POSITION {0.0, -PI/4, PI/2, -PI/4} // initial pos[rad] joints in initial joint control (space of the joints)
#define TETIS_TRAJ 4

/* ARDUINO BOARD CONFIG */
#define SPI_CS_PIN  53 // Uno: 10, Mega: 53
#define USB_BAUDRATE 921600 // USB serial baudrate, used for debugging & warning (9600,115200,921600)
#define CAN_BAUDRATE CAN_1000KBPS // Can network baudrate


/* TETIS SPECIFIC DATA */
#define E2  (float)52.55
#define E3  (float)320.0
#define E4  (float)225.0
#define E5  (float)167.25
#define M3  (float)55.775
#define M4  (float)55.775
#define M5  (float)57.0

/* TRANSMIT PDO 1 SETTINGS */
#define TPDO1_IN_TIME 1000 // inhibit time =  TPDO1_IN_TIME * 10exp(-6)s
#define TPDO1_TR_TYPE 1 // 255: async, 253: async on RTR only, 1:sync
#define PDO_READ_TIMEOUT 10000 // time(ms) to send new SYNC if all PDOS nto received

/* RECEIVE PDO 1 SETTINGS */
#define RPDO1_IN_TIME 1000 // inhibit time =  TPDO1_IN_TIME * 10exp(-6)s
#define RPDO1_TR_TYPE 255 // 255: async, 1:sync

/* ERROR CONTROL PROTOCOL SETTINGS */
#define HEARBEAT_TIME 1000
#define HB_DELAY_ALWD 200 // Allowed delay[ms] in hearbeat receptions

/* GLOBAL VARIABLES */

extern HardwareSerial* serial; // ptr to serial port

extern long unsigned h; // Sampling time(ms) for the control loops
extern long unsigned tLastExec; // time(ms)loop was last executed
extern long unsigned tDelay; // delay from time iteration was supposed to start

extern Joystick* joystick;

extern CanNet canNet;
extern User user;

extern BluetoothJoystick btJoystick;
extern ShieldJoystick shJoystick;
extern Joystick* joystick;

extern MatlabSerial matlab;

class User{
  public:
    unsigned int controlType;
    bool debugMode;
    int useMode; // 0: PC, 1:BT Controller, 2: BT Terminal
    bool toMatlab;
    User(unsigned int ct = 4, bool dm = false,bool tm = false, bool um = 0) :
      controlType(ct),debugMode(dm),toMatlab(tm),useMode(um){}
};


#endif  // EPOS2CONTROL_H
