#ifndef ROBOT_STRATEGY_H_
#define ROBOT_STRATEGY_H_

#include "Header.h"


#define MAX_COMMAND 53.1
class RobotStrategy{
private:
    int id;
    float maxCommand;

    float command[2];
    bool stopped;

    btVector3 robotPos;
    btVector3 robotStepTarget;
    btVector3 linVel;
    btVector3 linAce;
    btVector3 localFront;
    btVector3 localRight; //Necessary to relative positions
    float angVel;
    float angAce;

    int localFunction;
	float maxVelocity;
	float maxTargetAngle;
	float actDistToTarget;
	btVector3 targetPos;

    bool motion;
    bool invertFront;
public:
	RobotStrategy(int id ,float maxCommand = MAX_COMMAND, float maxBallAngle = 20, float actDistToBall = 10);

    void setStandardMotion(bool motion);
    bool getStandardMotion() { return motion; }

    void setStopped(bool stopped);
    bool getStopped() { return stopped; }

    void updateCommand(float left, float right);
    float* getCommand() { return command; }

    void setPosition(btVector3 pos);
    btVector3 getPosition() { return robotPos; }
    btVector3 getLocalFront(){ return localFront; }
    btVector3 getLocalRight();

    int getId() { return id; }
    int getLocalFunction(){ return localFunction; }
    float getFieldAngle();
	float getPointAngle(btVector3 target);
	float getTargetAngle();
	float getTargetDistance();
	float getMaxAngToTarget(){ return maxTargetAngle; }
	float getMaxCommand(){ return maxCommand; }
	float getActDistToTarget(){ return actDistToTarget; }

	btVector3 getLinearVelocity(){ return linVel; }
    btVector3 getLinearAcceleration(){ return linAce; }
    float getAngularVelocity(){ return angVel; }
    float getAngularAcceleration() { return angAce; }

	void setLinearVelocity(btVector3 vel);
    void setLinearAcceleration(btVector3 ace);
    void setAngularVelocity(float vel);
    void setAngularAcceleration(float ace);

    void setLocalFront(btVector3 frontW);
    void setLocalRight(btVector3 rightW);


	btVector3 getTargetPosition(){ return targetPos; }
	void setTargetPosition(btVector3 target);

    btVector3 getStepTargetPosition(){ return robotStepTarget; }
    void setStepTargetPosition(btVector3 target);

	void setLocalFunction(int func);
    void invertLocalFront();
    bool getInvertLocalFront() { return invertFront; }
};

#endif
