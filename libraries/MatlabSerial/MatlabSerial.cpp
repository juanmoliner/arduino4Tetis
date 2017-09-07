#include "Arduino4Tetis.h"


void MatlabSerial :: resetTime(){
  tInitPlot = millis();
}

void MatlabSerial :: plotX(KinematicSystem* ks){
  static long unsigned tLastPlot;
  if(first){
    resetTime();
    first = false;
  }
  if(millis() - tLastPlot > MATLAB_PLOT_SAMPLE_T){
    //actually not necessary since only used inside timed main loop
    Serial.print("ToMatlabX: ");
    for(int i = 0; i < 4; i++){
      Serial.print(ks->xd[i], MATLAB_PREC); Serial.print(" ");
    }
    for(int i = 0; i < 4; i++){
      Serial.print(ks->x[i], MATLAB_PREC); Serial.print(" ");
    }
    Serial.print(millis() - tInitPlot); Serial.println(" ");
    tLastPlot = millis();
  }
}

void MatlabSerial ::plotQ(KinematicSystem* ks){
  static long unsigned tLastPlot;
  if(first){
    resetTime();
    first = false;
  }
  if(millis() - tLastPlot > MATLAB_PLOT_SAMPLE_T){
    //actually not necessary since only used inside timed main loop
    Serial.print("ToMatlabQ: ");
    for(int i = 0; i < 4; i++){
      Serial.print(ks->qd[i] * RADTODEG, MATLAB_PREC); Serial.print(" ");
    }
    for(int i = 0; i < 4; i++){
      Serial.print(ks->systemJoints[i]->q * RADTODEG, MATLAB_PREC); Serial.print(" ");
    }
    Serial.print(millis() - tInitPlot); Serial.println(" ");
    tLastPlot = millis();
  }
}

void MatlabSerial::plotU(KinematicSystem* ks){
  static long unsigned tLastPlot;
  if(first){
    resetTime();
    first = false;
  }
  if(millis() - tLastPlot > MATLAB_PLOT_SAMPLE_T){
    //actually not necessary since only used inside timed main loop
    Serial.print("ToMatlabU: ");
    for(int i = 0; i < 4; i++){
      Serial.print(ks->systemJoints[i]-> u * RADTODEG,MATLAB_PREC); Serial.print(" ");
    }
    Serial.print(millis() - tInitPlot); Serial.println(" ");
    tLastPlot = millis();
  }
}
