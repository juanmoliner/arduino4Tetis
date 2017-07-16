// CanNet.h
#ifndef CANNET_H
#define CANNET_H

class CanNet : public MCP_CAN{
  public:
    unsigned int numOfKinS;
    KinematicSystem** kinSystems;
    CanNet(KinematicSystem** ks, unsigned int num,byte cs = 53) : MCP_CAN(cs), kinSystems(ks), numOfKinS(num){      }
    void setupCan();
    void canListenerPDO();
    void canListenerSDO();
    void readPosition(Joint*, byte*);
    void checkHearbeat();
    void setupHearbeat(KinematicSystem*,unsigned int hbTime = 1000);
    void toAllNodesSdo(KinematicSystem*, byte*, bool ext = 0);
    void zeroTPDOs(KinematicSystem*);
    void zeroRPDOs(KinematicSystem*);
    void setupTPDO1(KinematicSystem*);
    void setupRPDO1(KinematicSystem*);
    void setupVelocityMode(KinematicSystem*);
    void printBuffer();
    void uSetSDO();
    void uSetPDO();
    unsigned int getNumOfJoints(unsigned int ksNum);
};


#endif
