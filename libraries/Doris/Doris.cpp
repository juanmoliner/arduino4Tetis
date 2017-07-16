bool Doris :: checkJointLimits(){
  return false;
}

bool Doris :: checkCollision(){
  return false;
}

void Doris :: updateDirectKinematics(){
  serial -> println("Doris:updateDirectKinematics()");
}

void Doris :: updateControl(unsigned int operMode){
  serial -> println("Doris:updateControl()");
  joystick -> read();
  systemJoints[0] -> u = -5 * joystick -> getX();
  systemJoints[3] -> u = -5 * joystick -> getX();
  systemJoints[1] -> u = 5 * joystick -> getX();
  systemJoints[2] -> u = 5 * joystick -> getX();


}
