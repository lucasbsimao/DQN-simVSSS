#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include "Header.h"
#include "Scenario.h"
#include "strategies/ModelStrategy.h"
#include "ArtificialIntelligence.h"

class Simulator{
    struct GameState{
        vector<RobotStrategy*> robotStrategiesTeam;
        vector<RobotStrategy*> robotStrategiesAdv;
        bool sameState;

        GameState():sameState(true){};
    };

private:
    const float timeStep = 1.f/60.f;
    const float handTime = 10.f;
    int numRobotsTeam;
    bool runningPhysics;

    GameState* gameState;
    int stratStep;
    int loopBullet;

	HandleGraphics* handleGraphics;
	Physics* physics;
	vector<ModelStrategy*> strategies;

	void updateWorld();
	btVector3 calcRelativePosition(btVector3 absPos, int attackDir);
	void calcRelativeWorld(vector<RobotStrategy*> robotStrategiesTeam, int attackDir);
	RobotStrategy* updateLocalPhysics(int id, RobotPhysics* bdRobot);

    void initWorld();
    void reinitPhysics();
public:
	Simulator();
	void runSimulator(int argc, char *argv[], ModelStrategy* strategyTeam, ModelStrategy* strategyAdv, ArtificialIntelligence* artInt);

	void *runPhysics();
	void *runGraphics();
	void *runStrategies();

	static void* runGraphics_thread(void* simulator){
        return ((Simulator*)simulator)->runGraphics();
	}

    static void* runPhysics_thread(void* simulator){
        return ((Simulator*)simulator)->runPhysics();
    }

    static void* runStrategies_thread(void* simulator){
        return ((Simulator*)simulator)->runStrategies();
    }

};

#endif
