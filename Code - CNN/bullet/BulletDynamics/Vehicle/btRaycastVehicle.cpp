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

#include "LinearMath/btVector3.h"
#include "btRaycastVehicle.h"

#include "BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h"
#include "BulletDynamics/ConstraintSolver/btJacobianEntry.h"
#include "LinearMath/btQuaternion.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "btVehicleRaycaster.h"
#include "btWheelInfo.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"

#define ROLLING_INFLUENCE_FIX


btRigidBody& btActionInterface::getFixedBody()
{
	static btRigidBody s_fixed(0,0,0);
	s_fixed.setMassProps(btScalar(0.),btVector3(btScalar(0.),btScalar(0.),btScalar(0.)));
	return s_fixed;
}

btRaycastVehicle::btRaycastVehicle(const btVehicleTuning& tuning,btRigidBody* chassis,	btVehicleRaycaster* raycaster )
:m_vehicleRaycaster(raycaster),
m_pitchControl(btScalar(0.))
{
    _suspensionVel = btVector3(0.,0.,0.);
	m_chassisBody = chassis;
	m_indexLeftAxis = 0;
	m_indexUpAxis = 2;
	m_indexForwardAxis = 1;
	defaultInit(tuning);

	for(int i = 0; i < 2; i++)
	{
		debugLinearVelocity[i] = btVector3(0,0,0);
		debugRel_pos[i] = btVector3(0,0,0);
	}

	m_rayMotion = 0.f;
	m_absCenterMassLinVel = btVector3(0.f,0.f,0.f);
	m_absCenterMassAngVel = btVector3(0.f,0.f,0.f);
	m_hitedCenterMassAngVel = btVector3(0.f,0.f,0.f);
	m_hitedCenterMassLinVel = btVector3(0.f,0.f,0.f);
}


void btRaycastVehicle::defaultInit(const btVehicleTuning& tuning)
{
	(void)tuning;
	m_currentVehicleSpeedKmHour = btScalar(0.);
	m_steeringValue = btScalar(0.);

}

btRaycastVehicle::~btRaycastVehicle()
{
}


//
// basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed
//
btWheelInfo&	btRaycastVehicle::addWheel( const btVector3& connectionPointCS, const btVector3& wheelDirectionCS0,const btVector3& wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRadius,const btVehicleTuning& tuning, bool isFrontWheel)
{

	btWheelInfoConstructionInfo ci;

	ci.m_chassisConnectionCS = connectionPointCS;
	ci.m_wheelDirectionCS = wheelDirectionCS0;
	ci.m_wheelAxleCS = wheelAxleCS;
	ci.m_suspensionRestLength = suspensionRestLength;
	ci.m_wheelRadius = wheelRadius;
	ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
	ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
	ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
	ci.m_frictionSlip = tuning.m_frictionSlip;
	ci.m_bIsFrontWheel = isFrontWheel;
	ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
	ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;

	m_wheelInfo.push_back( btWheelInfo(ci));

	btWheelInfo& wheel = m_wheelInfo[getNumWheels()-1];

	updateWheelTransformsWS( wheel , false );
	updateWheelTransform(getNumWheels()-1,false);
	return wheel;
}

const btTransform&	btRaycastVehicle::getWheelTransformWS( int wheelIndex ) const
{
	btAssert(wheelIndex < getNumWheels());
	const btWheelInfo& wheel = m_wheelInfo[wheelIndex];
	return wheel.m_worldTransform;

}

void	btRaycastVehicle::updateWheelTransform( int wheelIndex , bool interpolatedTransform)
{

	btWheelInfo& wheel = m_wheelInfo[ wheelIndex ];
	updateWheelTransformsWS(wheel,interpolatedTransform);
	btVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
	const btVector3& left = wheel.m_raycastInfo.m_wheelAxleWS;
	btVector3 fwd = up.cross(left);
	fwd = fwd.normalize();
//	up = right.cross(fwd);
//	up.normalize();

	//rotate around steering over de wheelAxleWS
	btScalar steering = wheel.m_steering;

	btQuaternion steeringOrn(up,steering);//wheel.m_steering);
	btMatrix3x3 steeringMat(steeringOrn);

	btQuaternion rotatingOrn(left,-wheel.m_rotation);
	btMatrix3x3 rotatingMat(rotatingOrn);

	btMatrix3x3 basis2(
		left[0],fwd[0],up[0],
		left[1],fwd[1],up[1],
		left[2],fwd[2],up[2]
	);

	wheel.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
	wheel.m_worldTransform.setOrigin(
		wheel.m_raycastInfo.m_hardPointWS + wheel.m_raycastInfo.m_wheelDirectionWS * wheel.m_raycastInfo.m_suspensionLength
	);
}

void btRaycastVehicle::resetSuspension()
{
	int i;
	for (i=0;i<m_wheelInfo.size();	i++)
	{
			btWheelInfo& wheel = m_wheelInfo[i];
			wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
			wheel.m_suspensionRelativeVelocity = btScalar(0.0);

			wheel.m_raycastInfo.m_contactNormalWS = - wheel.m_raycastInfo.m_wheelDirectionWS;
			//wheel_info.setContactFriction(btScalar(0.0));
			wheel.m_clippedInvContactDotSuspension = btScalar(1.0);
	}
}

void	btRaycastVehicle::updateWheelTransformsWS(btWheelInfo& wheel , bool interpolatedTransform)
{
	wheel.m_raycastInfo.m_isInContact = false;

	btTransform chassisTrans = getChassisWorldTransform();
	if (interpolatedTransform && (getRigidBody()->getMotionState()))
	{
		getRigidBody()->getMotionState()->getWorldTransform(chassisTrans);
	}

	wheel.m_raycastInfo.m_hardPointWS = chassisTrans( wheel.m_chassisConnectionPointCS );
	wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() *  wheel.m_wheelDirectionCS ;
	//cout << "\nDIRECTION X:\t" << wheel.m_wheelDirectionCS.getX() << "\tY:\t" << wheel.m_wheelDirectionCS.getY() <<  "\tZ:\t" << wheel.m_wheelDirectionCS.getZ() << endl;
	wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

bool btRaycastVehicle::rayCast(btWheelInfo& wheel)
{
	updateWheelTransformsWS( wheel,false);


	bool outDepth = false;

	btScalar raylen = wheel.getSuspensionRestLength()+wheel.m_wheelsRadius;

	btVector3 rayvector = wheel.m_raycastInfo.m_wheelDirectionWS * (raylen);
	const btVector3& source = wheel.m_raycastInfo.m_hardPointWS;

	wheel.m_raycastInfo.m_contactPointWS = source + rayvector;
	const btVector3& target = wheel.m_raycastInfo.m_contactPointWS;

	btScalar param = btScalar(0.);

	btVehicleRaycaster::btVehicleRaycasterResult	rayResults;

	btAssert(m_vehicleRaycaster);

	void* object = m_vehicleRaycaster->castRay(source,target,rayResults);

	wheel.m_raycastInfo.m_groundObject = 0;

	if (object)
	{
		param = rayResults.m_distFraction;

		wheel.m_raycastInfo.m_contactNormalWS  = rayResults.m_hitNormalInWorld;
		wheel.m_raycastInfo.m_isInContact = true;

        btScalar hitDistance = param*raylen;
		wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;

		wheel.m_raycastInfo.m_groundObject = &getFixedBody();///@todo for driving on dynamic/movable objects!;

        if(wheel.m_raycastInfo.m_suspensionLength >= 0) {
            outDepth = true;

            //wheel.m_raycastInfo.m_groundObject = object;
            //clamp on max suspension travel

            btScalar  minSuspensionLength = wheel.getSuspensionRestLength() - wheel.m_maxSuspensionTravelCm*btScalar(0.01);
            btScalar maxSuspensionLength = wheel.getSuspensionRestLength()+ wheel.m_maxSuspensionTravelCm*btScalar(0.01);

            if (wheel.m_raycastInfo.m_suspensionLength < minSuspensionLength)
            {
                wheel.m_raycastInfo.m_suspensionLength = minSuspensionLength;
            }
            if (wheel.m_raycastInfo.m_suspensionLength > maxSuspensionLength)
            {
                wheel.m_raycastInfo.m_suspensionLength = maxSuspensionLength;
            }

            wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;

            btVector3 chassis_velocity_at_contactPoint;
            btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS-getRigidBody()->getCenterOfMassPosition();

            chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

            btScalar projVel = getWorldVectorInLocalCoordinate( chassis_velocity_at_contactPoint).getY();

            wheel.m_suspensionRelativeVelocity = projVel;
        }
	}

	if(!outDepth){
		//put wheel info as in rest position
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = btScalar(0.0);
		wheel.m_raycastInfo.m_contactNormalWS = - wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = btScalar(1.0);
	}

	return outDepth;
}


const btTransform& btRaycastVehicle::getChassisWorldTransform() const
{
	/*if (getRigidBody()->getMotionState())
	{
		btTransform chassisWorldTrans;
		getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
		return chassisWorldTrans;
	}
	*/


	return getRigidBody()->getCenterOfMassTransform();
}


void btRaycastVehicle::updateVehicle( btScalar step )
{
    updateCollisionTransform(step);
	{
		for (int i=0;i<getNumWheels();i++)
		{
			updateWheelTransform(i,false);
		}
	}

	m_currentVehicleSpeedKmHour = btScalar(3.6) * getRigidBody()->getLinearVelocity().length();

	const btTransform& chassisTrans = getChassisWorldTransform();

	btVector3 forwardW (
		chassisTrans.getBasis()[0][m_indexForwardAxis],
		chassisTrans.getBasis()[1][m_indexForwardAxis],
		chassisTrans.getBasis()[2][m_indexForwardAxis]);

	if (forwardW.dot(getRigidBody()->getLinearVelocity()) < btScalar(0.))
	{
		m_currentVehicleSpeedKmHour *= btScalar(-1.);
	}

	//
	// simulate suspension
	//

	int i=0;
	for (i=0;i<m_wheelInfo.size();i++)
	{
		if(!rayCast( m_wheelInfo[i])){
		}
	}

	updateSuspension(step);

	for (i=0;i<m_wheelInfo.size();i++)
	{
		//apply suspension force
		btWheelInfo& wheel = m_wheelInfo[i];

		btScalar suspensionForce = wheel.m_wheelsSuspensionForce;

		if (suspensionForce > wheel.m_maxSuspensionForce)
		{
			suspensionForce = wheel.m_maxSuspensionForce;
		}
        //cout << endl << "IMPULSE X:\t" << wheel.m_raycastInfo.m_contactNormalWS.getY() << endl;
		btVector3 impulse = getUpVector() * suspensionForce * step;
		btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();

        //system("clear");
		//cout << impulse.getX() << "\t" << impulse.getY() << "\t" << impulse.getZ() << endl;
        stringstream ss;
        ss << "Suspension " << i;
        string desc = ss.str();
        m_wheelInfo[i].m_velocityLocalSuspension = impulse*getRigidBody()->getInvMass();
		getRigidBody()->applyImpulse(impulse, relpos, desc);
        //calculate the suspension velocity influence on the general velocity

	}

	updateFriction( step);

	for (i=0;i<m_wheelInfo.size();i++)
	{
		btWheelInfo& wheel = m_wheelInfo[i];
		btVector3 relpos = wheel.m_raycastInfo.m_hardPointWS - getRigidBody()->getCenterOfMassPosition();
		btVector3 vel = getRigidBody()->getVelocityInLocalPoint(relpos);

		//btVector3 vel(0.,0.,0.);

		if (wheel.m_raycastInfo.m_isInContact)
		{
			const btTransform&	chassisWorldTransform = getChassisWorldTransform();

			btVector3 fwd (
				chassisWorldTransform.getBasis()[0][m_indexForwardAxis],
				chassisWorldTransform.getBasis()[1][m_indexForwardAxis],
				chassisWorldTransform.getBasis()[2][m_indexForwardAxis]);

			btScalar proj = fwd.dot(wheel.m_raycastInfo.m_contactNormalWS);
			fwd -= wheel.m_raycastInfo.m_contactNormalWS * proj;

			btScalar proj2 = fwd.dot(vel);

			wheel.m_deltaRotation = (proj2 * step) / (wheel.m_wheelsRadius);
			wheel.m_rotation += wheel.m_deltaRotation;

		} else
		{
			wheel.m_rotation += wheel.m_deltaRotation;
		}

		wheel.m_deltaRotation *= btScalar(0.99);//damping of rotation when not in contact

	}
}

void    btRaycastVehicle::updateCollisionTransform(btScalar timeStep){
    m_hitedCenterMassLinVel = m_chassisBody->getLinearVelocity()-m_absCenterMassLinVel;
    m_hitedCenterMassAngVel = m_chassisBody->getAngularVelocity()-m_absCenterMassAngVel;
}

void	btRaycastVehicle::setSteeringValue(btScalar steering,int wheel)
{
	btAssert(wheel>=0 && wheel < getNumWheels());

	btWheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_steering = steering;
}

btScalar	btRaycastVehicle::getSteeringValue(int wheel) const
{
	return getWheelInfo(wheel).m_steering;
}


void	btRaycastVehicle::applyEngineForce(btScalar force, int wheel)
{
	btAssert(wheel>=0 && wheel < getNumWheels());
	btWheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_engineForce = force;
}


const btWheelInfo&	btRaycastVehicle::getWheelInfo(int index) const
{
	btAssert((index >= 0) && (index < 	getNumWheels()));

	return m_wheelInfo[index];
}

btWheelInfo&	btRaycastVehicle::getWheelInfo(int index)
{
	btAssert((index >= 0) && (index < 	getNumWheels()));

	return m_wheelInfo[index];
}

void btRaycastVehicle::setBrake(btScalar brake,int wheelIndex)
{
	btAssert((wheelIndex >= 0) && (wheelIndex < 	getNumWheels()));
	getWheelInfo(wheelIndex).m_brake = brake;
}

void btRaycastVehicle::calcSuspensionVelocity()
{
    btVector3 rel_pos = getWheelInfo(3).m_raycastInfo.m_contactNormalWS - getRigidBody()->getCenterOfMassPosition();
}

void btRaycastVehicle::updateSuspension(btScalar deltaTime)
{
	(void)deltaTime;

	btScalar chassisMass = btScalar(1.) / m_chassisBody->getInvMass();

	for (int w_it=0; w_it<getNumWheels(); w_it++)
	{
		btWheelInfo &wheel_info = m_wheelInfo[w_it];

		if ( wheel_info.m_raycastInfo.m_isInContact )
		{
			btScalar force;
			//	Spring
			{
				btScalar	susp_length	= wheel_info.getSuspensionRestLength();
				btScalar	current_length = wheel_info.m_raycastInfo.m_suspensionLength;

				btScalar length_diff = (susp_length - current_length);

				force = wheel_info.m_suspensionStiffness * length_diff;// * wheel_info.m_clippedInvContactDotSuspension;
			}

			// Damper
			{
				btScalar projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
				{
					btScalar	susp_damping;
					if ( projected_rel_vel < btScalar(0.0) )
					{
						susp_damping = wheel_info.m_wheelsDampingCompression;
					}
					else
					{
						susp_damping = wheel_info.m_wheelsDampingRelaxation;
					}
					force -= susp_damping * projected_rel_vel;
				}
			}

			// RESULT
			wheel_info.m_wheelsSuspensionForce = force * chassisMass;
			if (wheel_info.m_wheelsSuspensionForce < btScalar(0.))
			{
				wheel_info.m_wheelsSuspensionForce = btScalar(0.);
			}
		}
		else
		{
			wheel_info.m_wheelsSuspensionForce = btScalar(0.0);
		}
	}

}

struct btWheelContactPoint
{
	btRigidBody* m_body0;
	btRigidBody* m_body1;
	btVector3	m_frictionPositionWorld;
	btVector3	m_frictionDirectionWorld;
	btScalar	m_jacDiagABInv;
	btScalar	m_maxImpulse;


	btWheelContactPoint(btRigidBody* body0,btRigidBody* body1,const btVector3& frictionPosWorld,const btVector3& frictionDirectionWorld, btScalar maxImpulse)
		:m_body0(body0),
		m_body1(body1),
		m_frictionPositionWorld(frictionPosWorld),
		m_frictionDirectionWorld(frictionDirectionWorld),
		m_maxImpulse(maxImpulse)
	{
		btScalar denom0 = body0->computeImpulseDenominator(frictionPosWorld,frictionDirectionWorld);
		btScalar denom1 = body1->computeImpulseDenominator(frictionPosWorld,frictionDirectionWorld);
		btScalar	relaxation = 1.f;
		m_jacDiagABInv = relaxation/(denom0+denom1);
	}
};

btScalar sideFrictionStiffness2 = btScalar(1.0);
void	btRaycastVehicle::updateFriction(btScalar	timeStep)
{

    //calculate the impulse, so that the wheels don't move sidewards
    int numWheel = getNumWheels();
    if (!numWheel)
        return;

    m_forwardWS.resize(numWheel);
    m_axle.resize(numWheel);
    m_forwardImpulse.resize(numWheel);

    int numWheelsOnGround = 0;


    //collapse all those loops into one!
    for (int i=0;i<getNumWheels();i++)
    {
        btWheelInfo& wheelInfo = m_wheelInfo[i];
        class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;
        if (groundObject)
            numWheelsOnGround++;

        m_forwardImpulse[i] = btScalar(0.);

    }

    {
        for (int i=0;i<getNumWheels();i++)
        {

            btWheelInfo& wheelInfo = m_wheelInfo[i];

            class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

            if (groundObject)
            {
                const btTransform& wheelTrans = getWheelTransformWS( i );

                btMatrix3x3 wheelBasis0 = wheelTrans.getBasis();
                m_axle[i] = btVector3(
                    wheelBasis0[0][m_indexLeftAxis],
                    wheelBasis0[1][m_indexLeftAxis],
                    wheelBasis0[2][m_indexLeftAxis]);

                const btVector3& surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
                btScalar proj = m_axle[i].dot(surfNormalWS);
                m_axle[i] -= surfNormalWS * proj;
                m_axle[i] = m_axle[i].normalize();

                m_forwardWS[i] = surfNormalWS.cross(m_axle[i]);
                m_forwardWS[i].normalize();
            }
        }
    }
	btScalar fwdFactor = 0.5;

	bool sliding = false;
	{
		for (int wheel =2;wheel <getNumWheels();wheel++)
		{
			btWheelInfo& wheelInfo = m_wheelInfo[wheel];
			class btRigidBody* groundObject = (class btRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

			btScalar	rollingFriction = 0.f;

			if (groundObject)
			{
				rollingFriction = wheelInfo.m_engineForce* timeStep;
			}

			//switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)
			m_forwardImpulse[wheel] = btScalar(0.);
			m_wheelInfo[wheel].m_skidInfo= btScalar(1.);

			if (groundObject)
			{
				m_wheelInfo[wheel].m_skidInfo= btScalar(1.);

				btScalar maximp = wheelInfo.m_wheelsSuspensionForce * timeStep * wheelInfo.m_frictionSlip;
				btScalar maximpSide = maximp;

				btScalar maximpSquared = maximp * maximpSide;


				m_forwardImpulse[wheel] = rollingFriction;//wheelInfo.m_engineForce* timeStep;

				btScalar x = (m_forwardImpulse[wheel] ) * fwdFactor;
				btScalar y = 0.f;//(m_sideImpulse[wheel] ) * sideFactor;

				btScalar impulseSquared = (x*x + y*y);

				if (impulseSquared > maximpSquared)
				{
					sliding = true;

					btScalar factor = maximp / btSqrt(impulseSquared);

					m_wheelInfo[wheel].m_skidInfo *= factor;
				}
			}
		}
	}

		if(!m_wheelInfo[rightWheel].m_raycastInfo.m_groundObject && !m_wheelInfo[leftWheel].m_raycastInfo.m_groundObject){
            m_forwardImpulse[rightWheel] = 0.f;
            m_forwardImpulse[leftWheel] = 0.f;
		}

        btVector3 hitedSideVelocityWS(0,0,0);
        BulletObject* robotBtObj = (BulletObject*)m_chassisBody->getUserPointer();
		if(robotBtObj && robotBtObj->hit){
            updateLocalComponentsHited();

            float modHitedSideVelocity = getWorldVectorInLocalCoordinate(btVector3(m_hitedCenterMassLinVel.getX(),0,m_hitedCenterMassLinVel.getZ())).getZ();

            if(modHitedSideVelocity < 0) hitedSideVelocityWS = modHitedSideVelocity*getLeftVector();
            else hitedSideVelocityWS = modHitedSideVelocity*getRightVector();
        }

		float dvLocalWheel[2];
		btVector3 distSteeringWh[2];
		// apply the velocities
		{
			for (int wheel = 2;wheel<getNumWheels() ; wheel++)
			{
				btWheelInfo& wheelInfo = m_wheelInfo[wheel];

                btVector3 relPos = wheelInfo.m_raycastInfo.m_contactPointWS - m_chassisBody->getCenterOfMassPosition();
                btVector3 velWheelXZ = btVector3(m_chassisBody->getVelocityInLocalPoint(relPos).getX(),0,m_chassisBody->getVelocityInLocalPoint(relPos).getZ());
                float modVelWheelXZ = velWheelXZ.length();

                //cout << endl << "modVelWheelXZ = " << modVelWheelXZ << endl;
                //cout << endl << "wheelInfo.m_localVelocity = " << wheelInfo.m_localVelocity << endl;

                dvLocalWheel[wheel-2] = m_forwardImpulse[wheel]*m_chassisBody->getInvMass();
				//cout << endl << "DIFF RAY "<< wheel << ":\t" << dvLocalWheel[wheel-2] << endl;

                wheelInfo.m_localVelocity += dvLocalWheel[wheel-2];
                //cout << endl << "VEL RAY "<< wheel << ":\t" << wheelInfo.m_localVelocity << endl;
                distSteeringWh[wheel-2] = wheelInfo.m_raycastInfo.m_contactPointWS;
			}
			//cout << endl << "A LEFT WHEEL:\t" << m_wheelInfo[leftWheel].m_localVelocity << "\tA RIGHT WHEEL:\t" << m_wheelInfo[rightWheel].m_localVelocity << endl;

            btVector3 vecRelDistStWh = (distSteeringWh[0] - distSteeringWh[1]);
            float distRelSteeringWh = vecRelDistStWh.length();
            btStraight strRayVelocity;
            if(m_wheelInfo[leftWheel].m_localVelocity > m_wheelInfo[rightWheel].m_localVelocity){
                strRayVelocity.calculateStraight( btVector3(0, 0,0), btVector3(m_wheelInfo[leftWheel].m_localVelocity -m_wheelInfo[rightWheel].m_localVelocity, 0,distRelSteeringWh));
                m_rayMotion =  strRayVelocity.getInclination()*m_wheelInfo[rightWheel].m_localVelocity+distRelSteeringWh/2;
                m_centripetalVelocity = getRightVector();
            }else{
                strRayVelocity.calculateStraight( btVector3(0, 0,0), btVector3(m_wheelInfo[rightWheel].m_localVelocity-m_wheelInfo[leftWheel].m_localVelocity, 0,distRelSteeringWh));
                m_rayMotion =  strRayVelocity.getInclination()*m_wheelInfo[leftWheel].m_localVelocity+distRelSteeringWh/2;
                m_centripetalVelocity = getLeftVector();
            }

            btVector3 suspLinVelocity;
            suspLinVelocity = btVector3(0,m_chassisBody->getLinearVelocity().getY(),0);

            float modCenterMassLinVelXZ = m_rayMotion/strRayVelocity.getInclination();

            float linearMovement = m_wheelInfo[leftWheel].m_localVelocity - m_wheelInfo[rightWheel].m_localVelocity;
            if(linearMovement == 0) modCenterMassLinVelXZ = m_wheelInfo[leftWheel].m_localVelocity;

            m_absCenterMassLinVel = suspLinVelocity+modCenterMassLinVelXZ*getForwardVector();

            modCenterMassLinVelXZ = getLinearVelocityXZ().length();

			//cout << "Lin Vel : " << modCenterMassLinVelXZ << endl;

            m_chassisBody->setLinearVelocity(m_absCenterMassLinVel);

            float centAceleleration = modCenterMassLinVelXZ*modCenterMassLinVelXZ/m_rayMotion;
            float modCentVelocity = centAceleleration*timeStep;

            m_centripetalVelocity = modCentVelocity*m_centripetalVelocity-hitedSideVelocityWS;

            if(m_rayMotion == 0.f){
				 m_centripetalVelocity = btVector3(0,0,0);
			}

            updateTransformCentripetalForce(timeStep);

            float centerMassAngY = modCenterMassLinVelXZ/m_rayMotion;

			

            btVector3 relLocalVelocity = getWorldVectorInLocalCoordinate(m_chassisBody->getLinearVelocity());

            if(m_rayMotion == 0.f){
                centerMassAngY = m_wheelInfo[rightWheel].m_localVelocity/(distRelSteeringWh/2);
            }else{
                if(relLocalVelocity.getX() > 0){
                    if(m_wheelInfo[leftWheel].m_localVelocity > m_wheelInfo[rightWheel].m_localVelocity)
                        centerMassAngY = -centerMassAngY;
                }else{
                    if(m_wheelInfo[leftWheel].m_localVelocity < m_wheelInfo[rightWheel].m_localVelocity)
                        centerMassAngY = -centerMassAngY;
                }
            }

			//cout << "VEL ANG RAY : " << centerMassAngY << endl;

            m_absCenterMassAngVel = btVector3(m_chassisBody->getAngularVelocity().getX(),0,m_chassisBody->getAngularVelocity().getZ())+getUpVector()*centerMassAngY;// btVector3(m_chassisBody->getAngularVelocity().getX(),centerMassAngY,m_chassisBody->getAngularVelocity().getZ());

            m_chassisBody->setAngularVelocity(m_absCenterMassAngVel);

            debugDrawProperties();
			//cout << "---------------------------" << endl;
		}
}

void btRaycastVehicle::updateLocalComponentsHited(){
    btVector3 chassisBodyXZ = btVector3(m_chassisBody->getLinearVelocity().getX(),0,m_chassisBody->getLinearVelocity().getZ());
    btVector3 hitedLinVelLocal = getWorldVectorInLocalCoordinate(chassisBodyXZ);
    float hitedAngVelLocal = m_chassisBody->getAngularVelocity().getY();
    btVector3 distRelSteeringWheel = m_wheelInfo[leftWheel].m_raycastInfo.m_contactPointWS - m_wheelInfo[rightWheel].m_raycastInfo.m_contactPointWS;
    float modDistRelStWheel = distRelSteeringWheel.length();

    m_absCenterMassLinVel = hitedLinVelLocal.getX()*getForwardVector();
    m_absCenterMassAngVel = m_chassisBody->getAngularVelocity();

    m_wheelInfo[leftWheel].m_localVelocity = hitedLinVelLocal.getX() - hitedAngVelLocal*modDistRelStWheel/2;
    m_wheelInfo[rightWheel].m_localVelocity = hitedLinVelLocal.getX() + hitedAngVelLocal*modDistRelStWheel/2;

    //cout << endl << "WHEEL LEFT:\t" << m_wheelInfo[leftWheel].m_localVelocity << "\tWHEEL LEFT:\t" << m_wheelInfo[rightWheel].m_localVelocity << endl;
}

btVector3 btRaycastVehicle::getWorldVectorInLocalCoordinate(btVector3 target){
    btVector3 targetUnit = target.normalized();

    btVector3 unitAxis[] = { getForwardVector(), getUpVector(), getRightVector() };
    float modToTarget = target.length();

    float components[3];
    for(int i = 0; i < 3; i++){
        float radToLocal = targetUnit.getX()*unitAxis[i].getX() + targetUnit.getY()*unitAxis[i].getY() + target.getZ()*unitAxis[i].getZ();

        if(radToLocal > 1.f) radToLocal = 1.f;
        if(radToLocal < -1.f) radToLocal = -1.f;

        components[i] = modToTarget*radToLocal;
    }

    return btVector3(components[0],components[1],components[2]);
}

void btRaycastVehicle::updateTransformCentripetalForce(btScalar timeStep){
    //btTransform chassisTrans = m_chassisBody->getWorldTransform();
    //btVector3 originalPos = chassisTrans.getOrigin();

    m_chassisBody->applyLinearVelocity(m_centripetalVelocity*timeStep);
}

void	btRaycastVehicle::debugDraw(btIDebugDraw* debugDrawer){
    int lenghtDebug = debugDrawer->getDebugMode().size();
    for(int i = 0; i < lenghtDebug; i++){
        if(debugDrawer->getDebugMode()[i] == btIDebugDraw::DBG_DrawLocalProperties){
            debugDrawProperties();
            m_chassisBody->debugLocalProperties(debugDrawer);
        }
    }

	for (int v=0;v<this->getNumWheels();v++){
		btVector3 wheelColor(0,1,1);
		if (getWheelInfo(v).m_raycastInfo.m_isInContact)
		{
			wheelColor.setValue(0,0,1);
		} else
		{
			wheelColor.setValue(1,0,1);
		}

		btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();

		btVector3 axle = btVector3(
			getWheelInfo(v).m_worldTransform.getBasis()[0][getLeftAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[1][getLeftAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[2][getLeftAxis()]);

		//debug wheels (cylinders)
		debugDrawer->drawLine(wheelPosWS,wheelPosWS+axle,wheelColor);
		debugDrawer->drawLine(wheelPosWS,getWheelInfo(v).m_raycastInfo.m_contactPointWS,wheelColor);

	}
}

void btRaycastVehicle::debugDrawProperties()
{
    btTransform chassisTrans = getChassisWorldTransform();
    btVector3 centerMass = m_chassisBody->getCenterOfMassPosition();
    btVector3 side = m_centripetalVelocity.normalized();

    btVector3 angUnit = debugLinearVelocity[1].normalized();

    glPushMatrix();
        glTranslatef(centerMass.getX(),centerMass.getY(),centerMass.getZ());
    glPopMatrix();
}


void* btDefaultVehicleRaycaster::castRay(const btVector3& from,const btVector3& to, btVehicleRaycasterResult& result)
{
//	RayResultCallback& resultCallback;

	btCollisionWorld::ClosestRayResultCallback rayCallback(from,to);

	m_dynamicsWorld->rayTest(from, to, rayCallback);

	if (rayCallback.hasHit())
	{

		const btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
        if (body && body->hasContactResponse())
		{
			result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
			result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
			result.m_hitNormalInWorld.normalize();
			result.m_distFraction = rayCallback.m_closestHitFraction;
			return (void*)body;
		}
	}
	return 0;
}

