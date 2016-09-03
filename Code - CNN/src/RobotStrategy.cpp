#include "RobotStrategy.h"

RobotStrategy::RobotStrategy(int id, float maxCommand, float maxTargetAngle, float actDistToTarget){
    this->id = id;
    this->maxCommand = maxCommand;
    this->maxTargetAngle = maxTargetAngle;
    this->actDistToTarget = actDistToTarget;
    this->localFunction = -1;
    this->motion = false;
    this->invertFront = false;
    this->stopped = false;

    for(int i = 0; i < 2; i++){
        command[i] = 0.f;
    }

    this->robotPos = btVector3(0,0,0);
    this->linVel = btVector3(0,0,0);
    this->linAce = btVector3(0,0,0);
    this->angVel = 0;
    this->angAce = 0;
    this->localFront = btVector3(0,0,0);
    this->targetPos = btVector3(0,0,0);
}

void RobotStrategy::setStandardMotion(bool motion){
    this->motion = motion;
}

void RobotStrategy::setStopped(bool stopped){
    this->stopped = stopped;
}

void RobotStrategy::updateCommand(float left, float right){
    if(!invertFront){
        command[0] = left;
        command[1] = right;
    }else{
        command[0] = -right;
        command[1] = -left;
    }
}

void RobotStrategy::setPosition(btVector3 pos){
    this->robotPos = pos;
}

void RobotStrategy::setLinearVelocity(btVector3 vel){
    this->linVel = vel;
}

void RobotStrategy::setLinearAcceleration(btVector3 ace){
    this->linAce = ace;
}

void RobotStrategy::setAngularVelocity(float vel){
    this->angVel = vel;
}

void RobotStrategy::setAngularAcceleration(float ace){
    this->angAce = ace;
}

void RobotStrategy::setLocalFront(btVector3 frontW){
    this->localFront = frontW;
    float rightX = getLocalFront().getZ() == 0 ? 0 : -getLocalFront().getZ();
    float rightZ = getLocalFront().getX() == 0 ? 0 : getLocalFront().getX();
    this->localRight = btVector3(rightX,0,rightZ);
}

void RobotStrategy::invertLocalFront(){
    this->localFront = -this->localFront;
    this->localRight = -this->localRight;
    invertFront = true;
}

void RobotStrategy::setLocalRight(btVector3 rightW){
    this->localRight = rightW;
}

btVector3 RobotStrategy::getLocalRight(){
    return localRight;
}

void RobotStrategy::setLocalFunction(int func){
    this->localFunction = func;
}

float RobotStrategy::getFieldAngle(){
    float angle = atan2(localFront.getZ(),localFront.getX())*180/PI;
    if(fabs(angle) > 360) angle = 0.00001f;
    return angle;
}

float RobotStrategy::getTargetAngle(){
    return getPointAngle(targetPos);
}

float RobotStrategy::getPointAngle(btVector3 targetPos){

    //Gets the unit vector between the robot and the target
    btVector3 rbPos = getPosition();
    btVector3 rbUnitVecX = getLocalFront();

    //cout << "FR : " << rbUnitVecX.getX() << " " << rbUnitVecX.getZ() << endl;

    btVector3 rbUnitVecZ = getLocalRight();

    btVector3 vecRobotTarget = targetPos-rbPos;

    vecRobotTarget.normalize();
    //cout << "VEC : " << vecRobotTarget.getX() << " " << vecRobotTarget.getZ() << endl;

    float targetZAngle = 180*vecRobotTarget.angle(rbUnitVecZ)/PI;

    float targetXAngle = 180*vecRobotTarget.angle(rbUnitVecX)/PI;
    
    float targetAng = 0.f;
    /*float targetProjX = vecRobotTarget.getX()/vecRobotTarget.length();
    float targetProjZ = vecRobotTarget.getZ()/vecRobotTarget.length();
    if(fabs(targetProjX) < 0.0001 ) targetProjX = 0.0001;
    if(fabs(targetProjZ) < 0.0001 ) targetProjZ = 0.0001;

    //Using the unit vector we get, by the dot product between two vectors, the angle between the robot's local Z and the target
    //If the angle is greater than 90 degrees means that the target is on the right of the robot, else it's on the left
    
    float targetZrad = targetProjX*rbUnitVecZ.getX() + targetProjZ*rbUnitVecZ.getZ();
    if(targetZrad > 1.f) targetZrad = 1.f;
    if(targetZrad < -1.f) targetZrad = -1.f;
    if(fabs(targetZrad) < 0.0001 ) targetZrad = 0.0001;
    float targetZAngle = 180*acos(targetZrad)/PI;
    if(targetZAngle == 0) targetZAngle = 0.00001;
    if(targetZAngle == 90) targetZAngle = 89.99999;

    //Same thing, but for the robot's local X
    float targetXrad = targetProjX*rbUnitVecX.getX() + targetProjZ*rbUnitVecX.getZ();
    if(targetXrad > 1.f) targetXrad = 1.f;
    if(targetXrad < -1.f) targetXrad = -1.f;
    if(fabs(targetXrad) < 0.0001 ) targetXrad = 0.0001;
    float targetXAngle = 180*acos(targetXrad)/PI;

    if(targetXAngle == 0) targetXAngle = 0.00001;
    if(targetXAngle == 90) targetXAngle = 89.99999;*/

    targetAng = targetXAngle;
    if(targetZAngle > 90) targetAng = -targetXAngle;

    return targetAng;
}

void RobotStrategy::setTargetPosition(btVector3 target){
    this->targetPos = target;
}

void RobotStrategy::setStepTargetPosition(btVector3 target){
    this->robotStepTarget = target;
}

float RobotStrategy::getTargetDistance(){
    btVector3 rbPos = getPosition();
    btVector3 dist = targetPos-rbPos;
    return dist.length();
}
