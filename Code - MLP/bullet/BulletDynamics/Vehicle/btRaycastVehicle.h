/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
*/
#ifndef BT_RAYCASTVEHICLE_H
#define BT_RAYCASTVEHICLE_H

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "btVehicleRaycaster.h"
class btDynamicsWorld;
#include "LinearMath/btAlignedObjectArray.h"
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"
#include <iostream>
#include <sstream>
#include "LinearMath/btStraight.h"
#include "Sir.h"

#include <GL/glut.h>
#include <math.h>

using namespace std;

class btVehicleTuning;

///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
class btRaycastVehicle : public btActionInterface
{
		btAlignedObjectArray<btVector3>	m_forwardWS;
		btAlignedObjectArray<btVector3>	m_axle;
		btAlignedObjectArray<btScalar>	m_forwardImpulse;

		///backwards compatibility
		int	m_userConstraintType;
		int	m_userConstraintId;

        //Debug Variables
		btVector3 debugLinearVelocity[2];
		btVector3 debugRel_pos[2];

public:
	class btVehicleTuning
		{
			public:

			btVehicleTuning()
				:m_suspensionStiffness(btScalar(5.88)),
				m_suspensionCompression(btScalar(0.83)),
				m_suspensionDamping(btScalar(0.88)),
				m_maxSuspensionTravelCm(btScalar(500.)),
				m_frictionSlip(btScalar(10.5)),
				m_maxSuspensionForce(btScalar(6000.))
			{
			}
			btScalar	m_suspensionStiffness;
			btScalar	m_suspensionCompression;
			btScalar	m_suspensionDamping;
			btScalar	m_maxSuspensionTravelCm;
			btScalar	m_frictionSlip;
			btScalar	m_maxSuspensionForce;

		};
private:

	btScalar	m_tau;
	btScalar	m_damping;
	btVehicleRaycaster*	m_vehicleRaycaster;
	btScalar		m_pitchControl;
	btScalar	m_steeringValue;
	btScalar m_currentVehicleSpeedKmHour;
	btVector3 _suspensionVel;

	btRigidBody* m_chassisBody;

	int m_indexLeftAxis;
	int m_indexUpAxis;
	int	m_indexForwardAxis;

	float m_rayMotion;
    btVector3 m_absCenterMassLinVel;
    btVector3 m_absCenterMassAngVel;
    btVector3 m_centripetalVelocity;

    btVector3 m_hitedCenterMassAngVel;
    btVector3 m_hitedCenterMassLinVel;

	void defaultInit(const btVehicleTuning& tuning);
	void calcSuspensionVelocity();

public:

    enum idWheels{
        frontWheel = 0,
        backWheel = 1,
        leftWheel = 2,
        rightWheel = 3,
    };
	//constructor to create a car from an existing rigidbody
	btRaycastVehicle(const btVehicleTuning& tuning,btRigidBody* chassis,	btVehicleRaycaster* raycaster );

	virtual ~btRaycastVehicle() ;


	///btActionInterface interface
	virtual void updateAction( btCollisionWorld* collisionWorld, btScalar step)
	{
        (void) collisionWorld;
		updateVehicle(step);
	}


	///btActionInterface interface
	void	debugDraw(btIDebugDraw* debugDrawer);

	void debugDrawProperties();

	const btTransform& getChassisWorldTransform() const;

	bool rayCast(btWheelInfo& wheel);

	virtual void updateVehicle(btScalar step);

	void resetSuspension();

	btScalar	getSteeringValue(int wheel) const;

	void	setSteeringValue(btScalar steering,int wheel);


	void	applyEngineForce(btScalar force, int wheel);

	const btTransform&	getWheelTransformWS( int wheelIndex ) const;

	void	updateWheelTransform( int wheelIndex, bool interpolatedTransform = true );

//	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, const btVector3& hitPoint, const btVector3& hitNormal,btScalar depth);

	btWheelInfo&	addWheel( const btVector3& connectionPointCS0, const btVector3& wheelDirectionCS0,const btVector3& wheelAxleCS,btScalar suspensionRestLength,btScalar wheelRadius,const btVehicleTuning& tuning, bool isFrontWheel);

	inline int		getNumWheels() const {
		return int (m_wheelInfo.size());
	}

	btAlignedObjectArray<btWheelInfo>	m_wheelInfo;


	const btWheelInfo&	getWheelInfo(int index) const;

	btWheelInfo&	getWheelInfo(int index);

	void	updateWheelTransformsWS(btWheelInfo& wheel , bool interpolatedTransform = true);


	void setBrake(btScalar brake,int wheelIndex);

	void	setPitchControl(btScalar pitch)
	{
		m_pitchControl = pitch;
	}

	void	updateSuspension(btScalar deltaTime);

	virtual void	updateFriction(btScalar	timeStep);

	void updateLocalComponentsHited();

	void updateCollisionTransform(btScalar timeStep);

	void updateTransformCentripetalForce(btScalar timeStep);

	btVector3 getWorldVectorInLocalCoordinate(btVector3 target);

	inline btRigidBody* getRigidBody()
	{
		return m_chassisBody;
	}

	const btRigidBody* getRigidBody() const
	{
		return m_chassisBody;
	}

	inline int	getLeftAxis() const
	{
		return m_indexLeftAxis;
	}
	inline int getUpAxis() const
	{
		return m_indexUpAxis;
	}

	inline int getForwardAxis() const
	{
		return m_indexForwardAxis;
	}


	///Worldspace forward vector
	btVector3 getForwardVector() const
	{
		const btTransform& chassisTrans = getChassisWorldTransform();

		btVector3 forwardLocal (
			  chassisTrans.getBasis()[0][m_indexForwardAxis],
			  chassisTrans.getBasis()[1][m_indexForwardAxis],
			  chassisTrans.getBasis()[2][m_indexForwardAxis]);

		return forwardLocal;
	}

	btVector3 getLeftVector() {
        const btTransform& chassisTrans = getChassisWorldTransform();

		btVector3 leftLocal (
			  chassisTrans.getBasis()[0][m_indexLeftAxis],
			  chassisTrans.getBasis()[1][m_indexLeftAxis],
			  chassisTrans.getBasis()[2][m_indexLeftAxis]);

		return leftLocal;
	}

	btVector3 getRightVector(){
        btVector3 rightLocal = btVector3(-getLeftVector().getX(),-getLeftVector().getY(),-getLeftVector().getZ());
        return rightLocal;
	}

	btVector3 getUpVector() {
        const btTransform& chassisTrans = getChassisWorldTransform();

		btVector3 upLocal (
			  chassisTrans.getBasis()[0][m_indexUpAxis],
			  chassisTrans.getBasis()[1][m_indexUpAxis],
			  chassisTrans.getBasis()[2][m_indexUpAxis]);

		return upLocal;
	}

	///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
	btScalar	getCurrentSpeedKmHour() const
	{
		return m_currentVehicleSpeedKmHour;
	}

	btVector3    getLinearVelocityXZ(){
        btVector3 vecLinearVelocityXZ = btVector3(m_absCenterMassLinVel.getX(),0,m_absCenterMassLinVel.getZ());
        return vecLinearVelocityXZ;
	}

	virtual void	setCoordinateSystem(int leftIndex,int upIndex,int forwardIndex)
	{
		m_indexLeftAxis = leftIndex;
		m_indexUpAxis = upIndex;
		m_indexForwardAxis = forwardIndex;
	}


	///backwards compatibility
	int getUserConstraintType() const
	{
		return m_userConstraintType ;
	}

	void	setUserConstraintType(int userConstraintType)
	{
		m_userConstraintType = userConstraintType;
	};

	void	setUserConstraintId(int uid)
	{
		m_userConstraintId = uid;
	}

	int getUserConstraintId() const
	{
		return m_userConstraintId;
	}

};

class btDefaultVehicleRaycaster : public btVehicleRaycaster
{
	btDynamicsWorld*	m_dynamicsWorld;
public:
	btDefaultVehicleRaycaster(btDynamicsWorld* world)
		:m_dynamicsWorld(world)
	{
	}

	virtual void* castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result);

};


#endif //BT_RAYCASTVEHICLE_H

