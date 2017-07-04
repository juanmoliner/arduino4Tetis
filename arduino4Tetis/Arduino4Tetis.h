#ifndef ARDUINO4TETIS_H
#define ARDUINO4TETIS_H

#include <mcp_can.h>
#include <Arduino.h>
#include "ShieldJoystick.h"

/* HELPFUL UNIT CONVERSION */
#define QDTORAD (2*PI/(4 * ENCODER_CPR)) // rad / quadcounts
#define RADTOQD ((4 * ENCODER_CPR)/ 2*PI)
#define RADSTORPM (60/(2*PI)) // rpm /(rad/s)
#define RPMTORADS ((2*PI)/60)
#define RADTODEG (180/PI) // grad/rad
#define DEGTORAD (PI/180)

/* BLUETOOTH */
#define BT_MODE // wether bluetooth is used
#define BT_BAUDRATE 57600

/* MATLAB PLOTTING MODE*/
#define TO_MATLAB // if defined sends data to be plotted to Matlab
#define MATLAB_PREC 4 // number of decimals to send to Matlab
#define MATLAB_PLOT_SAMPLE_T 60 //Matlab plotting sample time(ms)

/*  TEST MODE */
#define SIMU_MODE // simulation mode (overrides TWO_MOTOR_TEST)
// #define TWO_MOTOR_TEST // 2 motors simulated + 2 real

/* DEBUG MODE */
#undef DEBUG_MODE // take off warning
// #define DEBUG_MODE
// #define DEBUG_PREC 3 // number of decimals to show in debug mode

/* CONTROL PARAMETERS*/
#define FORGET_JLMITS_COLIS // define to take of joint and colision limits
#define FORGET_SATURATION // define to take off saturation limits
#define SAMP_TIME 40 // Sampling time (ms) for the control loops
#define PERMT_DELAY 0 // Acceptable delay (ms) for each loop
#define GAMMA 10 // gamma constant of first order filter in joystick control
#define KJ {1.0, 1.0, 1.0, 1.0} // joint control proportional gain
#define KP {1.0, 1.0, 1.0, 1.0} // actuator control proportional gain
#define K_JOYSTICK  50 // joystick gain
#define INIT_Q_MAX_ERROR 0.3 // max error(qd-q)[grad] allowed in initial joint control out of singular
#define INIT_X_MAX_ERROR  1 // max error(xd-x)[mm or rad] allowed in initial pos control
#define Q_INIT_POSITION {0.0, -PI/4, PI/2, -PI/4} // initial pos[rad] joints in initial joint control (space of the joints)
#define X_INIT_POSITION {550, 57, -100, 0}  // initial pos[mm] actuator if initSimuPosition() used (space of the actuator)

/* ACTUATOR SYSTEM CONFIG */
#define NUMBER_OF_JOINTS 4
#define NODEID_OFFSET 5 // Offset of nodes ID(1st should be ID=1)
#define NODEID_MAPPING {9,8,7,6}
#define MOTOR_REDUCTION {100, 100, 100, 100} // 21:1
#define ENCODER_CPR 2000 // incremental enconder counts per revolution (assumes all system encodes have same cpr)
#define EPOS_POLARITY {-1,1,-1,-1} // 1 if positive theta is hourly
#define MAX_VELOCITY {620, 630, 625, 625} // velocity limit move [rpm @ motor]
#define MAX_ACCELERATION {600, 750, 650, 650} // max acceleration [rpm/s @ motor]
#define JOINTS_INIT_VALS {0.0, -PI/2, 0.0, 0.0}
#define J_LIMIT_OVP_ALLWD 0.1 * DEGTORAD // allowed overpass of joint limit [rad]


/* ARDUINO BOARD CONFIG */
#define SPI_CS_PIN  53 // Uno: 10, Mega: 53
#define USB_BAUDRATE 921600 // USB serial baudrate, used for debugging & warning (9600,115200,921600)
#define CAN_BAUDRATE CAN_500KBPS // Can network baudrate


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
#define PDO_READ_TIMEOUT 5 // time(ms) to send new SYNC if all PDOS nto received

/* RECEIVE PDO 1 SETTINGS */
#define RPDO1_IN_TIME 1000 // inhibit time =  TPDO1_IN_TIME * 10exp(-6)s
#define RPDO1_TR_TYPE 255 // 255: async, 1:sync

/* ERROR CONTROL PROTOCOL SETTINGS */
#define HEARBEAT_TIME 1000 // Hearbeat protocol producer time (multiple of 1ms)
#define HB_DELAY_ALWD 200 // Allowed delay[ms] in hearbeat receptions

/* GLOBAL VARIABLES */
extern long unsigned h; // Sampling time(ms) for the control loops
extern long unsigned tLastExec; // time(ms)loop was last executed
extern long unsigned tDelay; // delay from time iteration was supposed to start

extern long unsigned lastHeartbeat[NUMBER_OF_JOINTS]; // last time[ms] hearbeat message was received

extern long unsigned tLastPlot; // time(ms) last plot in Matlab

extern bool initialControl; // wether inital position control has already been made

extern long unsigned tInitPlot; // time(ms) to set as zero in Matlab plot

extern unsigned int nodeIDMapping[NUMBER_OF_JOINTS]; // Node id correspondig to each joint

extern char eposPolarity[NUMBER_OF_JOINTS]; // 1 if positive theta is hourly
extern unsigned int motorReduction[NUMBER_OF_JOINTS];
extern float maxVelocity[NUMBER_OF_JOINTS]; // velocity limit move [rpm @ motor]
extern float maxAcceleration[NUMBER_OF_JOINTS];// max acceleration [rpm/s @ motor]

extern float kp[NUMBER_OF_JOINTS]; // actuator control proportional gain
extern float xd[NUMBER_OF_JOINTS]; // desired position of the actuator at h(space of the joints)
extern float xddot[NUMBER_OF_JOINTS]; // UNUSED???desired velocity of the actuator at h(space of the joints)

extern float qd[NUMBER_OF_JOINTS]; // desired position[rad] for each joint (space of the joints)

extern float x[NUMBER_OF_JOINTS]; // actual position of the actuator (space of the actuator)
extern float kj[NUMBER_OF_JOINTS]; // joint control proportional gain
extern float q[NUMBER_OF_JOINTS]; // actual position[rad] of each joint(space of the joints)

extern float u[NUMBER_OF_JOINTS]; // control variable angular velocity [rad/s]
extern float ubar[NUMBER_OF_JOINTS]; // auxiliary control variable angular velocity [rad/s]
extern float error[NUMBER_OF_JOINTS]; // error  defined as: e = xd -x

extern long  qEncOffset[NUMBER_OF_JOINTS]; // initial read [qd] of incremental encoder
extern float qinit[NUMBER_OF_JOINTS];  // initial angle of each joint. MAKE SURE ALL JOINTS START IN THIS POSITION

extern float c1, s1, c2, s2, c3, s3, c4, s4, c23, s23, c34, s34, c234, s234; // Tetis specific variables
extern float J0[NUMBER_OF_JOINTS][NUMBER_OF_JOINTS]; // Jacobian at the base (joint 0)
extern float JN[NUMBER_OF_JOINTS][NUMBER_OF_JOINTS]; // Jacobian at the actuator (joint n)

#ifdef BT_MODE
extern BluetoothJoystick joystick;
#else
extern ShieldJoystick joystick;
#endif

extern byte SYNC[2];
extern byte OPERATIONAL[2];
extern byte PREOPERATIONAL[2];

extern byte SHUTDOWN[8];
extern byte FAULTRESET[8];
extern byte ONANDENABLE[8];

extern MCP_CAN CAN;    // Set CS pin

/* FUNCTION PROTOTYPES */
bool tetisCheckColision();
bool tetisCheckJointLimits();
void uSet();
void CANListener();
void plotQInMatlab();
void plotXInMatlab();
void plotUInMatlab();


#endif  // EPOS2CONTROL_H
