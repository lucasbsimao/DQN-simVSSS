#ifndef ROBOT_PHYSICS_H_
#define ROBOT_PHYSICS_H_

#define MAX_ACCELERATION 50.0*SCALE_WORLD

#include "Header.h"
#include "BulletDynamics/Vehicle/btRaycastVehicle.h"

class RobotPhysics{
private:
    int contador;

	Color clrTeam;
    Color clrPlayer;

	float sizeRobot;
	float mass;
	float wheelWidth;
	float wheelStRadius;
	float wheelFrRadius;

	float suspensionStiffness;
	float suspensionDamping;
	float suspensionCompression;
	float wheelFriction;
	float rollInfluence;
	btScalar sRestLengthSt;
	btScalar sRestLengthFr;

	float freeWheelAng;

	float forceLeftWheel;
	float forceRightWheel;
	float timeStep;

	btVector3 point;
	btRigidBody* bdRobot;

	btRaycastVehicle::btVehicleTuning tuning;
	btVehicleRaycaster* raycaster;
	btRaycastVehicle* raycast;

	void setRobotState();
	void calcProportionalVelocity(float iniWheelVel[2], float endWheelVel[2]);
public:
	RobotPhysics(){}
	~RobotPhysics();

	RobotPhysics(btVector3 point, float mass, btRigidBody* bdRobot,Color plrColor, Color teamColor,float wheelWidth = 0.6, float wheelStRadius = 2, float wheelFrWheel = 0.75 ,float sizeRobot = 8.0);
	void buildRobot(btDynamicsWorld* world);
	void updateRobot(float* speed);

	btVector3 getLocalUnitVecX();
	btVector3 getLocalUnitVecZ();
	btVector3 getPosition();
	Color getColorTeam() { return clrTeam; }
	Color getColorPlayer() { return clrPlayer; }

	float getSteeringWheelRadius() { return wheelStRadius; }
	float getFreeWheelRadius() { return wheelFrRadius; }
	float getWheelWidth() { return wheelWidth; }

	btRaycastVehicle* getRaycast() { return raycast; }
	btRigidBody* getRigidBody() { return bdRobot; }

	void setTimeStep(float timeStep);
};

#endif
