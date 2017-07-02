#include "Arduino4Tetis.h"

void plotXInMatlab(){
  static long unsigned tLastPlot;
  if(millis() - tLastPlot > MATLAB_PLOT_SAMPLE_T){
    //actually not necessary since only used inside timed main loop
    Serial.print("ToMatlabX: ");
    for(int i = 0; i < NUMBER_OF_JOINTS; i++){
      Serial.print(xd[i], MATLAB_PREC); Serial.print(" ");
    }
    for(int i = 0; i < NUMBER_OF_JOINTS; i++){
      Serial.print(x[i], MATLAB_PREC); Serial.print(" ");
    }
    Serial.print(millis() - tInitPlot); Serial.print(" ");
    Serial.println();
    tLastPlot = millis();
  }
}

void plotQInMatlab(){
  static long unsigned tLastPlot;
  if(millis() - tLastPlot > MATLAB_PLOT_SAMPLE_T){
    //actually not necessary since only used inside timed main loop
    Serial.print("ToMatlabQ: ");
    for(int i = 0; i < NUMBER_OF_JOINTS; i++){
      Serial.print(qd[i] * RADTODEG, MATLAB_PREC); Serial.print(" ");
    }
    for(int i = 0; i < NUMBER_OF_JOINTS; i++){
      Serial.print(q[i] * RADTODEG, MATLAB_PREC); Serial.print(" ");
    }
    Serial.print(millis() - tInitPlot); Serial.print(" ");
    Serial.println();
    tLastPlot = millis();
  }
}

void plotUInMatlab(){
  static long unsigned tLastPlot;
  if(millis() - tLastPlot > MATLAB_PLOT_SAMPLE_T){
    //actually not necessary since only used inside timed main loop
    Serial.print("ToMatlabU: ");
    for(int i = 0; i < NUMBER_OF_JOINTS; i++){
      Serial.print(u[i] * RADTODEG,MATLAB_PREC); Serial.print(" ");
    }
    Serial.print(millis() - tInitPlot); Serial.print(" ");
    Serial.println();
    tLastPlot = millis();
  }
}
