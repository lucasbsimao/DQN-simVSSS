#include "Physics.h"

Physics::Physics(int numTeams){
    this->numTeams = numTeams;
    this->numRobotsTeam = NUM_ROBOTS_TEAM;

	collisionConfig = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfig);
    broadphase = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    world = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfig);
    world->setGravity(btVector3(0,-9.81*SCALE_WORLD,0));

    glDebugDrawer = new GLDebugDrawer();
    world->setDebugDrawer(glDebugDrawer);
    gContactAddedCallback = callBackHitFunc;
    
    createWorld();

    registBodies();
    refreshingWorld = false;

    usleep(100);
    srand(time(NULL));
}

Physics::~Physics(){
    deleteWorldObj();

    if(collisionConfig) delete collisionConfig;
    if(dispatcher) delete dispatcher;
    if(broadphase) delete broadphase;
    if(solver) delete solver;
    if(world) delete world;
}

void Physics::deleteWorldObj(){
    
    int i;
    for (i=world->getNumCollisionObjects()-1; i>=0 ;i--)
    {
        btCollisionObject* obj = world->getCollisionObjectArray()[i];
        world->removeCollisionObject( obj );
        delete obj;
    }

    /*for(int i = 0; i<bodies.size();i++){
        delete bodies[i];
    }

    for(int i = 0;i<genRobots.size();i++){
        delete genRobots[i];
    }*/
}

void Physics::registBodies(){
    
    addFloor();

    btVector3 posTeam1[] = {btVector3(15,4,SIZE_DEPTH- 55),btVector3(35,4,30),btVector3(55,4,45)};
    btVector3 posTeam2[] = {btVector3(SIZE_WIDTH-15,4,55),btVector3(SIZE_WIDTH-25,4,SIZE_DEPTH - SIZE_DEPTH/2.5 + 20),btVector3(SIZE_WIDTH-55,4,85)};
    //Create robots here
    //Team 1
    for(int i = 0;i < gameDepth;i++){
        for(int j = 0;j < gameWidth;j++){
            if(map.at(i).at(j) == 1){
                addRobot(Color(0.9,0.9,0.9),btVector3(i*SCALE_MAP+SCALE_MAP/2,2,j*SCALE_MAP+SCALE_MAP/2),btVector3(0,90,0),8,0.25,Color(1,0,0),Color(1.0,1.0,0));
            }
            if(map.at(i).at(j) == 5){
                addWall(Color(0,0,0),btVector3(i*SCALE_MAP+SCALE_MAP/2,2.5,j*SCALE_MAP+SCALE_MAP/2),7,5,7,0);
            }
            if(map.at(i).at(j) == 2){
                addBall(2.5,btVector3(i*SCALE_MAP+SCALE_MAP/2,2.5,j*SCALE_MAP+SCALE_MAP/2),0.08);
            }
        }
    }

    //addWall(Color(0,0,0),btVector3(SIZE_WIDTH/2,7.5,-5),SIZE_WIDTH,15,10,0);
    //addWall(Color(0,0,0),btVector3(SIZE_WIDTH/2,7.5,SIZE_DEPTH+5),SIZE_WIDTH,15,10,0);
    //addWall(Color(0,0,0),btVector3(-5,7.5,SIZE_DEPTH/2),10,15,SIZE_DEPTH,0);
    //addWall(Color(0,0,0),btVector3(SIZE_WIDTH+5,7.5,SIZE_DEPTH/2),10,15,SIZE_DEPTH,0);
}

void Physics::stepSimulation(float timeW,float subStep, float timeStep){
    if(!verifyWorld())
        world-> stepSimulation(timeW, subStep, timeStep);
}

bool Physics::verifyWorld(){
    //Verifying if strategy achieve a terminal state
    if(refreshingWorld) return true;

    //Verifying if robot hits the ball
    for(int i=0;i<bodies.size();i++){
		if(bodies[i]->name.compare("ball") == 0){
            BulletObject* blObj = (BulletObject*)bodies[i]->body->getUserPointer();
            if(blObj->hit){
                return true;
            }
			break;
		}
	}
    return false;
}

void Physics::reinitWorld(){
    refreshingWorld = true;
    
    createWorld();
    refreshPositions();

    refreshingWorld = false;
}

void Physics::refreshPositions(){
    btRigidBody* robotBody;
    btRigidBody* ballBody;
    vector<btRigidBody*> wallsBodies; 

    for(int i = 0; i < bodies.size(); i++){
        if(bodies[i]->name.compare("ball") == 0){
            ballBody = bodies[i]->body;
        }
        if(bodies[i]->name.compare("robot") == 0){
            robotBody = bodies[i]->body;
        }
        if(bodies[i]->name.compare("wall") == 0){
            wallsBodies.push_back(bodies[i]->body);
        }
    }

    int wall = 0;

    for(int i = 0;i < gameDepth;i++){
        for(int j = 0;j < gameWidth;j++){
            if(map.at(i).at(j) == 1){
                repositioningBody(robotBody, i, j);
            }
            if(map.at(i).at(j) == 5){
                repositioningBody(wallsBodies.at(wall), i, j);
                wall++;
            }
            if(map.at(i).at(j) == 2){
                repositioningBody(ballBody, i, j);
            }
        }
    }
}

void Physics::repositioningBody(btRigidBody* body, int i, int j){
    btVector3 pos = btVector3(i*SCALE_MAP+SCALE_MAP/2,2.5,j*SCALE_MAP+SCALE_MAP/2);
    btTransform t;
	body->getMotionState()->getWorldTransform(t);

    t.setOrigin(pos);

    btQuaternion rotquat;
    rotquat = rotquat.getIdentity();
    rotquat.setEuler(PI/2,0,0);

    t.setRotation(rotquat);

    body->setWorldTransform(t);
    body->getMotionState()->setWorldTransform(t);

    body->setLinearVelocity(btVector3(0.f,0.f,0.f));
    body->setAngularVelocity(btVector3(0.f,0.f,0.f));

    body->clearForces();
}

void Physics::setRefreshingWorld(bool refresh){
    refreshingWorld = refresh;
}

bool Physics::callBackHitFunc(btManifoldPoint& cp,const btCollisionObjectWrapper* obj1,int id1,int index1,const btCollisionObjectWrapper* obj2,int id2,int index2){
    string prefix = "robot";

    const btCollisionObjectWrapper* wrappers[] = {obj1, obj2};

    for(int i = 0; i < 2; i++){
        BulletObject* btObj = (BulletObject*)wrappers[i]->getCollisionObject()->getUserPointer();

        string name = btObj->name;
        if (!name.compare(0, prefix.size(), prefix) || name == "ball"){
            btObj->hit = true;
        }
    }

    return false;
}


btVector3 Physics::getBallPosition(){
	btVector3 ballPos;
	for(int i=0;i<bodies.size();i++){
		if(bodies[i]->name.compare("ball") == 0){
			btTransform t;
			bodies[i]->body->getMotionState()->getWorldTransform(t);
            ballPos = t.getOrigin();
			break;
		}
	}
	return ballPos;
}

void Physics::startDebug(){
    world->debugDrawWorld();
}

void Physics::setDebugWorld(int debugMode){
    vector<int> debugDrawMode;
    ((GLDebugDrawer*)world-> getDebugDrawer())->setDrawScenarioMode(true);
    switch (debugMode){
        case 0:{
            debugDrawMode.push_back(btIDebugDraw::DBG_NoDebug);
            world->getDebugDrawer()->setDebugMode(debugDrawMode);
        }break;
        case 1:{
            debugDrawMode.push_back(btIDebugDraw::DBG_DrawLocalProperties);
            debugDrawMode.push_back(btIDebugDraw::DBG_DrawWireframe);
            world->getDebugDrawer()->setDebugMode(debugDrawMode);
            ((GLDebugDrawer*)world-> getDebugDrawer())->setDrawScenarioMode(false);
        }break;
        case 2:{
            debugDrawMode.push_back(btIDebugDraw::DBG_DrawWireframe);
            debugDrawMode.push_back(btIDebugDraw::DBG_DrawLocalProperties);
            world-> getDebugDrawer()->setDebugMode(debugDrawMode);
        }break;
    }
}

btRigidBody* Physics::addFloor(){
	string name = "floor";
    btStaticPlaneShape* plane = new btStaticPlaneShape(btVector3(0,1,0),0);
    btRigidBody* body = addGenericBody(plane,name,Color(0.0,0.0,0.0),btVector3(1.0,0.0,0.0),0);
    return body;
}

btRigidBody* Physics::addBall(float rad,btVector3 pos,float mass)
{
    string name = "ball";
    btSphereShape* ball = new btSphereShape(rad);
    btRigidBody* body = addGenericBody(ball,name,Color(1.0,0.0,0.0), pos, mass);
    return body;
}

btRigidBody* Physics::addWall(Color clr, btVector3 pos,float width, float height, float depth, float mass){
    string name = "wall";
    btBoxShape* wall = new btBoxShape(btVector3(width/2.0,height/2.0,depth/2.0));
    btRigidBody* body = addGenericBody(wall,name,clr,pos,mass);
    return body;
}

btRigidBody* Physics::addCorner(Color clr, btVector3 pos,float width, float height,btVector3 rotation){
    float mass = 0.f;
    float depth = 0.01f;
    btVector3 rotRad = rotation*PI/180;
    string name = "corner";
    btBoxShape* corner = new btBoxShape(btVector3(width/2.0,height/2.0,depth/2.0));
    btRigidBody* body = addGenericBody(corner,name,clr,pos,mass,rotRad);
    return body;
}

RobotPhysics* Physics::addRobot(Color clr, btVector3 pos, btVector3 rotation,float sizeRobot, float mass,Color colorPlayer,Color colorTeam){
    btBoxShape* modelShape = new btBoxShape(btVector3(sizeRobot/2,sizeRobot/2,sizeRobot/2));
    btCompoundShape* compound = new btCompoundShape();

    btTransform localTrans;
    localTrans.setIdentity();
    localTrans.setOrigin(btVector3(0.0,4.0,0));

    compound->addChildShape(localTrans,modelShape);

    btTransform transRobot;
    transRobot.setIdentity();
    transRobot.setOrigin(btVector3(pos.getX(),pos.getY(),pos.getZ()));
    if(rotation.length() != 0){
        rotation *= PI/180;
        float rad = rotation.length();
        btVector3 axis = rotation.normalize();
        btQuaternion quat(axis,rad);
        transRobot.setRotation(quat);
    }

    btVector3 localInertia(0,0,0);
    compound->calculateLocalInertia(mass,localInertia);

    btMotionState* robotMotState = new btDefaultMotionState(transRobot);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,robotMotState,compound,localInertia);
    btRigidBody* bdRobot = new btRigidBody(cInfo);
    bdRobot->setCollisionFlags(bdRobot->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

    bdRobot -> setLinearVelocity(btVector3(0,0,0));
    bdRobot -> setAngularVelocity(btVector3(0,0,0));

    world->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
        bdRobot-> getBroadphaseHandle(),
        world -> getDispatcher()
    );

    bdRobot->setIdDebug(1);

    bodies.push_back(new BulletObject(bdRobot,"robot",clr));
    bodies[bodies.size()-1]->halfExt = modelShape->getHalfExtentsWithMargin();
    bdRobot->setUserPointer(bodies[bodies.size()-1]);

    world->addRigidBody (bdRobot);

    RobotPhysics* localRobot = new RobotPhysics(pos,0.2,bdRobot,colorPlayer,colorTeam);
    localRobot->buildRobot(world);

    world-> addVehicle(localRobot-> getRaycast());

    genRobots.push_back(localRobot);
    return localRobot;
}

btRigidBody* Physics::addGenericBody(btCollisionShape* shape,string name,Color clr, btVector3 pos, float mass,btVector3 rotation){

	btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(pos.getX(),pos.getY(),pos.getZ()));

    if(rotation.length() != 0){
        float rad = rotation.length();
        btVector3 axis = rotation.normalize();
        btQuaternion quat(axis,rad);
        t.setRotation(quat);
    }

    btVector3 inertia(0,0,0);
    if(mass!=0.0)
        shape->calculateLocalInertia(mass,inertia);

    btMotionState* motion=new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(mass,motion,shape,inertia);
    btRigidBody* body=new btRigidBody(info);

    bodies.push_back(new BulletObject(body,name,clr));
    body->setUserPointer(bodies[bodies.size()-1]);
    world->addRigidBody(body);
    return body;
}

void Physics::createWorld(){
    gameWidth = SIZE_WIDTH/SCALE_MAP;
    gameDepth = SIZE_DEPTH/SCALE_MAP;

    int numObstacles = gameWidth*gameDepth/6.5; 

    int typeMap = rand()%4;

    map.clear();
    for(int i = 0; i < gameDepth;i++){
        vector<float> temp;
        for(int j = 0; j < gameWidth;j++){
            temp.push_back(0);
        }
        map.push_back(temp);
    }

    int coordX = rand()%(gameWidth);
    int coordY = 0;

    if(typeMap == 0) map.at(0).at(coordX) = 1;
    if(typeMap == 1) map.at(coordX).at(0) = 1;
    if(typeMap == 2) map.at(coordX).at(gameDepth-1) = 1;
    if(typeMap == 3) map.at(gameDepth-1).at(coordX) = 1;

    usleep(100);
    coordX = rand()%(gameWidth);
    if(typeMap == 0) map.at(gameDepth-1).at(coordX) = 2;
    if(typeMap == 1) map.at(coordX).at(gameDepth-1) = 2;
    if(typeMap == 2) map.at(coordX).at(0) = 2;
    if(typeMap == 3) map.at(0).at(coordX) = 2;

    for(int i = 0; i < numObstacles;i++){
        
        do{
           coordX = rand()%(gameWidth);
           usleep(1.f/60.f);
           coordY = rand()%(gameDepth); 
        }while(!validObstacle(coordX,coordY));

        map.at(coordY).at(coordX) = 5;
    }

    /*for(int i = 0; i < gameWidth;i++){
        if(map.at(0).at(i) != 5){
            map.at(0).at(i) = 1;
            break;
        }
    }
    
    for(int i = 0; i < gameWidth;i++){
        if(map.at(gameDepth-1).at(i) != 5){
            map.at(gameDepth-1).at(gameWidth-i-1) = 2;
            break;
        }
    }*/

    cout <<"^ X "<< endl;
    cout <<"| "<< endl;
    cout <<"| "<< endl;
    for(int i= gameDepth -1; i >= 0;i--){
        cout << "| ";
        for(int j= 0; j < gameWidth;j++){
            cout << map.at(i).at(j) << " ";
        }   
        cout << endl;  
    }
    cout << "------------------------------------> Y " << endl;
    
}

bool Physics::validObstacle(int coordX, int coordY){
    bool valid = true;
    for(int i= coordY-1; i <= (coordY+1);i++){
        for(int j= coordX-1; j <= (coordX+1);j++){
            if(i >= 0 && j >= 0 && i < (gameWidth) && j < (gameDepth)){
                if(map.at(i).at(j) != 0){
                    valid = false;   
                }
            }
        }   
    }
    
    return valid;
}
