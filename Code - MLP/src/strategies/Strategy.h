#ifndef STRATEGY_H_
#define STRATEGY_H_

#define SIMULATOR_TO_VSS 1.0
#define TURN_ROBOT 1.0

#include "Header.h"
#include "ModelStrategy.h"
#include "ArtificialIntelligence.h"

class Strategy : public ModelStrategy{
private:
	bool reachingPoint;

	float areaTarget;
    float sizeRobot;
    float distProjBallWall;
    float maxDiffRobotPos;
    int timeToFullHistory;

    bool runningRL;
    bool reachDeepState;

    ArtificialIntelligence* artInt;

    btVector3 ballTarget;

	float timeStep;

	//DQN constants
	float reward;
	bool actAimTerminal;
	bool reachTerminal;

	int futStateX;
	int futStateZ;

	Map mapVision;
	Map mapReward;

    void updateStrategiesHistory();

	void controlLocalStrategy(RobotStrategy* robotStrategy);

	void updateActionMotion(RobotStrategy* robotStrategy);

	float handleLocalMaxVelocity(RobotStrategy* robotStrategy);

	void printMDPState();
	void calculateNextTarget(RobotStrategy* robotStrategy);
	bool calculateEnvReward(Map mapReward, int futPosX, int futPosZ);

	void printTermMapVision();

	void processMapVision();
public:
	Strategy();
	~Strategy();

	void reinitStrategy();
	bool agentReachTerminal() { return reachTerminal; }

	void runStrategy(vector<RobotStrategy*> robotStrategiesTeam, Map reward, bool followPolicy, bool saveNet);
};

#endif
