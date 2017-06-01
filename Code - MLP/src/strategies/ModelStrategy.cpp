#ifndef ModelStrategy_CPP_
#define ModelStrategy_CPP_

#include "ModelStrategy.h"

ModelStrategy::ModelStrategy(){
    this->targetVel = btVector3(0,0,0);
    this->targetAce = btVector3(0,0,0);
    attackDir = 1;
    deltaTime = 0.001f;
    numFramesPerSec = 30;

    begin_time  = clock();
}

void ModelStrategy::runStrategy(){
    deltaTime = float( clock () - begin_time ) /  CLOCKS_PER_SEC;
    begin_time = clock();

    if(strategyHistory.size() > numFramesPerSec) updateDynamics();

    for(int i = 0; i < robotStrategiesTeam.size();i++){
        updateTargetPosition(robotStrategiesTeam[i]);
    }

    updateStrategiesHistory();
}

void ModelStrategy::updateTargetPosition(RobotStrategy* robotStrategy){
    robotStrategy->setTargetPosition(targetPos);
}

void ModelStrategy::updateStrategiesHistory(){
    strategyHistory.clear();
    strategyHistory.push_back(this);
}

void ModelStrategy::setFramesSec(int numFramesSec){
    this->numFramesPerSec = numFramesSec;
}

void ModelStrategy::setAttackDir(int att){
    posGoalpost = btVector3(SIZE_WIDTH,0,SIZE_DEPTH/2);
    this->attackDir = att;
}

void ModelStrategy::updateDynamics(){
    for(int i = 0; i < robotStrategiesTeam.size();i++){
        updateLocalDynamics(robotStrategiesTeam[i],strategyHistory[strategyHistory.size()-1]->getRobotStrategiesTeam()[i]);
        updateLocalDynamics(robotStrategiesAdv[i],strategyHistory[strategyHistory.size()-1]->getRobotStrategiesAdv()[i]);
    }

    btVector3 diffTargetVel = targetVel - strategyHistory[strategyHistory.size()-1]->getTargetVelocity();
    targetAce = attackDir*diffTargetVel/deltaTime;

    btVector3 diffTargetPos = targetPos - strategyHistory[strategyHistory.size()-1]->getTargetPosition();
    targetVel = attackDir*diffTargetPos/deltaTime;
}

void ModelStrategy::updateLocalDynamics(RobotStrategy* curRbSt,RobotStrategy* oldRbSt){
    btVector3 diffVel = curRbSt->getLinearVelocity() - oldRbSt->getLinearVelocity();
    curRbSt->setLinearAcceleration(diffVel/deltaTime);

    btVector3 diffPos = curRbSt->getPosition() - oldRbSt->getPosition();
    curRbSt->setLinearVelocity(diffPos/deltaTime);

    float diffAng = curRbSt->getFieldAngle() - oldRbSt->getFieldAngle();
    curRbSt->setAngularVelocity(diffAng/deltaTime);

    float diffVelAng = curRbSt->getFieldAngle() - oldRbSt->getFieldAngle();
    curRbSt->setAngularAcceleration(diffVelAng/deltaTime);
}

#endif
