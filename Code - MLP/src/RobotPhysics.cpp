#include "RobotPhysics.h"

// 3 - RODA DIREITA
// 2 - RODA ESQUERDA

RobotPhysics::~RobotPhysics(){
    delete raycaster;
    delete raycast;
}

RobotPhysics::RobotPhysics(btVector3 point,
float mass,
btRigidBody* bdRobot,
Color plrColor,
Color teamColor,
float wheelWidth,
float wheelStRadius,
float wheelFrRadius,
float sizeRobot){
    int contador = 0;

    float compVal = 0.2;
    float dampVal = 0.3;

    this->sizeRobot = sizeRobot;
	this->point = point;
	this->mass = mass;
	this->bdRobot = bdRobot;
	this->wheelWidth = wheelWidth;
	this->wheelFrRadius = wheelFrRadius;
    this->wheelStRadius = wheelStRadius;

    this->clrPlayer = plrColor;
    this->clrTeam = teamColor;

	suspensionStiffness = 750.f;
	suspensionDamping = dampVal*2*sqrt(suspensionStiffness);
	suspensionCompression = compVal*2*sqrt(suspensionStiffness);
	wheelFriction = 10000.0f;
	rollInfluence = 1.0f;
	sRestLengthSt = btScalar(1.0f);
	sRestLengthFr = btScalar(1.0f);

    freeWheelAng = 0;
    forceLeftWheel = 0.f;
    forceRightWheel = 0.0f;
}

void RobotPhysics::buildRobot(btDynamicsWorld* world){
	if(sizeRobot <= 0 || mass <= 0){
		cout << "Your robot must have correct parameters to be built!" << endl;
        exit(EXIT_FAILURE);
	}



	btVector3 wheelDirection(0,-1,0);
    btVector3 wheelAxle(-1,0,0);
    int rightIndex = 0;
    int upIndex = 1;
    int forwardIndex = 2;

    raycaster = new btDefaultVehicleRaycaster(world);
    raycast = new btRaycastVehicle(tuning, bdRobot, raycaster);

    bdRobot->setActivationState(DISABLE_DEACTIVATION);

    float connectionHeight = 0.7f;
    float halfExt = sizeRobot/2;

    raycast -> setCoordinateSystem(rightIndex,upIndex,forwardIndex);

    bool isFrontWheel=true;
    btVector3 connectionPointFree(0,-0.8,-3.5);
    raycast -> addWheel(connectionPointFree,wheelDirection,wheelAxle,sRestLengthFr,wheelFrRadius,tuning,isFrontWheel);

    connectionPointFree= btVector3(0,-0.8,3.5);
    raycast -> addWheel(connectionPointFree,wheelDirection,wheelAxle,sRestLengthFr,wheelFrRadius,tuning,isFrontWheel);

    isFrontWheel = false;
    btVector3 connectionPointSide(halfExt+1.5*wheelWidth,connectionHeight,0);
    raycast -> addWheel(connectionPointSide,wheelDirection,wheelAxle,sRestLengthSt,wheelStRadius,tuning,isFrontWheel);

    connectionPointSide= btVector3(-halfExt-1.5*wheelWidth,connectionHeight,0);
    raycast -> addWheel(connectionPointSide,wheelDirection,wheelAxle,sRestLengthSt,wheelStRadius,tuning,isFrontWheel);

    for (int i = 0; i < raycast->getNumWheels(); i++)
    {
        btWheelInfo& wheel = raycast -> getWheelInfo(i);
        wheel.m_suspensionStiffness = suspensionStiffness;
        wheel.m_wheelsDampingRelaxation = suspensionDamping;
        wheel.m_wheelsDampingCompression = suspensionCompression;
        wheel.m_frictionSlip = wheelFriction;
        wheel.m_rollInfluence = rollInfluence;
    }
}

void RobotPhysics::updateRobot(float* speed){

    float endWheelVel[2];
    endWheelVel[0] = speed[0];
    endWheelVel[1] = speed[1];

    float iniWheelVel[2];
    for(int i = 2; i < raycast->getNumWheels();i++){
        iniWheelVel[i-2] = raycast->getWheelInfo(i).m_localVelocity;
    }

    calcProportionalVelocity(iniWheelVel,endWheelVel);
 
    int wheelIndex = 2;
    raycast->applyEngineForce(forceLeftWheel,wheelIndex);

    wheelIndex = 3;
    raycast->applyEngineForce(forceRightWheel,wheelIndex);

    contador++;
}

void RobotPhysics::calcProportionalVelocity(float iniWheelVel[2], float endWheelVel[2]){
    float timeLeft = 0.f;
    float diffVelocity[2];
    for(int i = 0; i< 2;i++){
        diffVelocity[i] = endWheelVel[i] - iniWheelVel[i];
        //cout << "ini: " << iniWheelVel[i] << endl;
        //cout << "diff: " << diffVelocity[i] << endl;
    }

    float deltaVelocity = MAX_ACCELERATION*timeStep;

    float percAceleration = 1.f;
    if(fabs(diffVelocity[0]) < fabs(diffVelocity[1])){
        timeLeft = fabs(diffVelocity[1])/(MAX_ACCELERATION);

        if(fabs(diffVelocity[1]) < deltaVelocity)
            percAceleration = fabs(diffVelocity[1])/deltaVelocity;

        forceRightWheel = MAX_ACCELERATION*percAceleration*mass;

        if(endWheelVel[1] < iniWheelVel[1]) forceRightWheel = -forceRightWheel;

        if(timeLeft == 0.f) timeLeft = 0.001f;
        float acLeftWheel = diffVelocity[0]/timeLeft;

        deltaVelocity = fabs(acLeftWheel)*timeStep;

        if(fabs(diffVelocity[0]) < deltaVelocity)
            percAceleration = fabs(diffVelocity[0])/deltaVelocity;

        forceLeftWheel = acLeftWheel*percAceleration*mass;
    }else{
        timeLeft = fabs(diffVelocity[0])/(MAX_ACCELERATION);

        if(fabs(diffVelocity[0]) < deltaVelocity)
            percAceleration = fabs(diffVelocity[0])/deltaVelocity;

        forceLeftWheel = MAX_ACCELERATION*percAceleration*mass;

        if(endWheelVel[0] < iniWheelVel[0]) forceLeftWheel = -forceLeftWheel;

        if(timeLeft == 0.f) timeLeft = 0.001f;

        float acRightWheel = diffVelocity[1]/timeLeft;

        deltaVelocity = fabs(acRightWheel)*timeStep;

        if(fabs(diffVelocity[1]) < deltaVelocity)
            percAceleration = fabs(diffVelocity[1])/deltaVelocity;

        forceRightWheel = acRightWheel*percAceleration*mass;
    }

    if(fabs(forceRightWheel) < 0.00001) forceRightWheel = 0.00001;
    if(fabs(forceLeftWheel) < 0.00001) forceLeftWheel = 0.00001;
}

btVector3 RobotPhysics::getPosition(){
    btTransform  transTemp;
    bdRobot->getMotionState()->getWorldTransform(transTemp);
    btVector3 posTemp =transTemp.getOrigin();
    btVector3 pos(posTemp.getX(),posTemp.getY(),posTemp.getZ());

    return pos;
}

btVector3 RobotPhysics::getLocalUnitVecX(){
    btVector3 vecTemp = raycast->getForwardVector();
    btVector3 vect(vecTemp.getX(),vecTemp.getY(),vecTemp.getZ());
    return vect;
}

btVector3 RobotPhysics::getLocalUnitVecZ(){
    btVector3 rightVector = raycast->getRightVector();

    btVector3 unitVecZ = btVector3(rightVector.getX(),rightVector.getY(),rightVector.getZ());
    return unitVecZ;
}

void RobotPhysics::setTimeStep(float timeStep){
    this->timeStep = timeStep;
}
