// Doris.h

#ifndef DORIS_H
#define DORIS_H

class Doris : public KinematicSystem {
   public:
    Doris(Joint** sj) : KinematicSystem(sj,4){
      name = "Doris";
    }
    void updateControl(unsigned int operMode);
    void updateDirectKinematics();
    bool checkCollision();
    bool checkJointLimits();
 };


#endif
