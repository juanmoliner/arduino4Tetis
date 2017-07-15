#ifndef MATLABSERIAL_H
#define MATLABSERIAL_H

#include "Arduino4Tetis.h"

class KinematicSystem;

class MatlabSerial{
  long unsigned sampleT;
  long unsigned tInitPlot;
  bool first = true;
  unsigned int precision;
  public:
    MatlabSerial(long unsigned st, unsigned int pr):
    sampleT(st),precision(pr){}
      void plotX(KinematicSystem*);
      void plotU(KinematicSystem*);
      void plotQ(KinematicSystem*);
      void resetTime();
};

#endif
