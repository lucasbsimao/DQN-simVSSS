#include "Strategy.h"

Strategy::Strategy(){
    areaTarget = 10.f*SIMULATOR_TO_VSS;
    distProjBallWall = 5.f*SIMULATOR_TO_VSS;
    maxDiffRobotPos = 3.5f*SIMULATOR_TO_VSS;    

    sizeRobot = 8*SIMULATOR_TO_VSS;
    attackDir = 1;
    deltaTime = 0.001f;
    timeToFullHistory = 0.f;
    numFramesPerSec = 30;
    ballTarget = btVector3(0,0,0);

    runningRL = false;
    reachDeepState = false;
    reachingPoint = false;

    artInt = new ArtificialIntelligence();

    begin_time  = clock();

    reward = 0;
    actAimTerminal = false;
    reachTerminal = false;
}

Strategy::~Strategy(){
    strategyHistory.clear();
}

void Strategy::runStrategy(vector<RobotStrategy*> robotStrategiesTeam, Map mapReward, bool followPolicy, bool saveNet){
    deltaTime = float( clock () - begin_time ) /  CLOCKS_PER_SEC;
    begin_time = clock();

    
    if(saveNet) artInt->saveNetwork();

    artInt->setFollowPolicy(followPolicy);

    printMDPState();

    this->robotStrategiesTeam = robotStrategiesTeam;

    this->mapReward = mapReward;

    processMapVision();

    if(strategyHistory.size() == numFramesPerSec){
        updateDynamics();
    }

    for(int i = 0; i < robotStrategiesTeam.size(); i++){
        controlLocalStrategy(robotStrategiesTeam[i]);
    }
    updateStrategiesHistory();
}

void Strategy::reinitStrategy(){
    reachingPoint = false;
    reachTerminal = false;
    actAimTerminal = false;
    reward = 0;
    //artInt = new ArtificialIntelligence();
    drawComponents.clear();
}

void Strategy::processMapVision(){
    Map mapVision;
    for(int i = 0; i < mapReward.size()+2;i++){
        vector<float> row;
        for(int j = 0; j < mapReward.at(0).size()+2;j++){
            if(i == 0 || i == mapReward.size()+1 || j == mapReward.at(0).size()+1 || j == 0)
                row.push_back(8);
            else
                if(mapReward.at(i-1).at(j-1) == 1)
                    row.push_back(0);
                else
                    row.push_back(mapReward.at(i-1).at(j-1));
            
        }
        mapVision.push_back(row);
    }

    this->mapVision = mapVision;
}

bool Strategy::calculateEnvReward(Map mapReward, int posX, int posZ){

    if(posX == mapReward.size() || posZ == mapReward.at(0).size() || posX == -1 || posZ == -1){
        this->reward = artInt->getMinReward();
        return true;
    }else if(mapReward.at(posX).at(posZ) == 1 || mapReward.at(posX).at(posZ) == 0 || mapReward.at(posX).at(posZ) == 5){
        this->reward = artInt->getStdReward();
        return false;
    }
    else if(mapReward.at(posX).at(posZ) == 2){
        this->reward = artInt->getMaxReward();
        return true;
    }else{
        cout << "Verify reward table! It is not returning any state reward." << endl;
    }


    return true;
}

void Strategy::updateStrategiesHistory(){
    strategyHistory.clear();
    strategyHistory.push_back(this);
}

void Strategy::printMDPState(){
    /*Map map = artInt->getMdp();
    float maxReward = artInt->getMaxReward();
    float impartState = artInt->getImpartialState();
    float minReward = artInt->getMinReward();

    vector<DrawComponents> listDrawComponents;
    for(int i = 0;i < map.size();i++){
        for(int j = 0;j < map.at(0).size();j++){
            Color color;
            if(map.at(i).at(j) < 0){
                float percColor = map.at(i).at(j)/minReward;
                if(percColor > 1) percColor = 1;
                color = Color(percColor, 0,0);
            }else{
                float percColor = map.at(i).at(j)/maxReward;
                if(percColor > 1) percColor = 1;
                color = Color(0, percColor,0);
            }
                
            if(map.at(i).at(j) == impartState) color = Color(0,0,1);

            btVector3 centerPt(-10+i*SCALE_MAP+SCALE_MAP/2,2.5,-10+j*SCALE_MAP+SCALE_MAP/2);
            vector<btVector3> listVectors;
            listVectors.push_back(centerPt + btVector3(5,0,5));
            listVectors.push_back(centerPt + btVector3(5,0,-5));
            listVectors.push_back(centerPt + btVector3(-5,0,-5));
            listVectors.push_back(centerPt + btVector3(-5,0,5));

            DrawComponents drawComp(listVectors, color);

            listDrawComponents.push_back(drawComp);
        }
    }

    drawComponents = listDrawComponents;*/
}

void Strategy::controlLocalStrategy(RobotStrategy* robotStrategy){
    calculateNextTarget(robotStrategy);
    updateActionMotion(robotStrategy);
}

void Strategy::calculateNextTarget(RobotStrategy* robotStrategy){
    int agentX = robotStrategy->getPosition().getX()/SCALE_MAP;
    int agentZ = robotStrategy->getPosition().getZ()/SCALE_MAP;

    mapVision.at(agentX+1).at(agentZ+1) = 1; 

    for(int i= mapVision.size() -1; i >= 0;i--){
        for(int j= 0; j < mapVision.at(0).size();j++){
            cout << mapVision.at(i).at(j) << " ";
        }   
        cout << endl;  
    }

    //verifying if is in negative part, because of round in int
    if(robotStrategy->getPosition().getX() < 0) agentX = -1;
    if(robotStrategy->getPosition().getZ() < 0) agentZ = -1;

    btVector3 futState((futStateX+0.5)*SCALE_MAP, 0, (futStateZ+0.5)*SCALE_MAP);

    if(artInt->getTakenActions() > 0){

        btVector3 distFutPoint = futState - robotStrategy->getPosition();
        if(distFutPoint.length() < 2.5)
            reachingPoint = false;
    }

    if(!reachingPoint){
        artInt->setMap(mapVision, reward, actAimTerminal);
       	int action = artInt->getActualAction();

        futStateX = agentX;
        futStateZ = agentZ;
        switch(action){
            case 0:{
                futStateX = agentX - 1;
            }break;
            case 1:{
                futStateZ = agentZ + 1;
            }break;
            case 2:{
                futStateX = agentX + 1;
            }break;
            case 3:{
                futStateZ = agentZ - 1;
            }break;
        }

        //Guarantees that terminal states will affect backpropagation
        if(actAimTerminal){
            if(!reachTerminal){
                reachTerminal = true;
                artInt->reinitMapProp();
                return;
            }
        }

        actAimTerminal = calculateEnvReward(mapReward, futStateX, futStateZ);

        if(!actAimTerminal){
            if( mapReward.at(futStateX).at(futStateZ) == 5){
                futStateX = agentX;
                futStateZ = agentZ;
            }
        }

        reachingPoint = true;
    }
    
    robotStrategy->setTargetPosition(btVector3((futStateX+0.5)*SCALE_MAP, 0, (futStateZ+0.5)*SCALE_MAP));
    
    //printTermMapVision();
}

void Strategy::printTermMapVision(){
    int posX = futStateX < 0 ? 0 : futStateX > (mapReward.size() -1) ? (mapReward.size() -1) : futStateX;
    int posZ = futStateZ < 0 ? 0 : futStateZ > (mapReward.size() -1) ? (mapReward.size() -1) : futStateZ;

    mapReward.at(posX).at(posZ) = 1;
    for(int i= mapReward.size() -1; i >= 0;i--){
        for(int j= 0; j < mapReward.at(0).size();j++){
            if(i != (mapReward.size()-posX) && j != posZ && mapReward.at(i).at(j) == 1) mapReward.at(i).at(j) = 0;
            cout << mapReward.at(i).at(j) << " ";
        }   
        cout << endl;  
    }
}

float Strategy::handleLocalMaxVelocity(RobotStrategy* robotStrategy){
	float perc;

	float distToBall = robotStrategy->getTargetDistance();
	float actDistBall = robotStrategy->getActDistToTarget();

	if(distToBall < actDistBall) perc = distToBall/actDistBall;
	else perc = 1;

    if(perc < 0.7) perc = 0.7;

	return perc;
}

void Strategy::updateActionMotion(RobotStrategy* robotStrategy){
	float maxAngToBall = robotStrategy->getMaxAngToTarget();
	float currAngToBall = robotStrategy->getPointAngle(robotStrategy->getTargetPosition());

	float maxVelocity = robotStrategy->getMaxCommand();

	float handMaxVel = maxVelocity*handleLocalMaxVelocity(robotStrategy);
	float percAng = 1 - fabs(currAngToBall)/maxAngToBall;
    if(percAng < 0.0) percAng = 0.0;
    if(percAng > 1) percAng = 1.0;

	float handLowVel = handMaxVel*percAng;

	if(fabs(currAngToBall) < maxAngToBall){
		if(currAngToBall < 0){
			robotStrategy->updateCommand(handLowVel,handMaxVel);
		}else{
			robotStrategy->updateCommand(handMaxVel,handLowVel);
		}
        robotStrategy->setStandardMotion(true);
	}else{
		if(currAngToBall < 0){
			robotStrategy->updateCommand(-maxVelocity,maxVelocity);
		}else{
			robotStrategy->updateCommand(maxVelocity,-maxVelocity);
		}
	}
}