# simVSSS and Reinforcement learning

This project is a branch of the original simVSSS. The objective was develop tests in a primitive tabular and a end-to-end version of Deep Q-Learning (DQN - [Deepmind blog post](https://deepmind.com/research/dqn/ "Deep Q-Learning")) using
the simVSSS platform.

## Deep Q-Learning

The DQN algorithm is an attempt to achieve the goal of beat high-human levens in play atari games. The principle of the algorithm is that you 
enter the raw of pixels present in screen to a neural network, and its the backprop is made by correcting rewards in each taken action. The algorithm
can be represented in the figure below.

![DQN](https://github.com/lucasbsimao/DQN-simVSSS/blob/master/images/nature_dqn.jpg "DQN")

The results achieved by the algorithm are very impressive, and the rank comparing both human level and DQN precision are the follows:

![DQN-rank](https://github.com/lucasbsimao/DQN-simVSSS/blob/master/images/rank_dqn_nature.jpg "DQN-rank")

## simVSSS and DQN

The original idea was implement original algorithm, using the pixels, to train a neural net in a tabular problem. But the PC used in this tests have an AMD Graphics Card. Beyond it, the original problem took arround 38 days to train, which is very difficult to achieve even with a simpler problem. 
So, the idea here is use the cpu library for deep learning tiny-dnn. There are two versions of the algorithms in this repository: CNN and MLP. Both uses tabular
method, but for the reasons given above only the MLP will achive good results in short time (arround 2 hours). The implemented idea is shown below.

| Algorithm        | Representation   |
| ------------- |:-------------:| 
| CNN      | ![DQN-CNN](https://github.com/lucasbsimao/DQN-simVSSS/blob/master/images/dqn-cnn.png "DQN-CNN") |
| MLP      | ![DQN-MLP](https://github.com/lucasbsimao/DQN-simVSSS/blob/master/images/dqn-mlp.png "DQN-MLP")      |

## Results

The results with the MLP algoritm was quite reasonable. But it is dificult to see this algorihms implemented for real games in short future.
The loss show us that the problem converges.

![DQN-COST](https://github.com/lucasbsimao/DQN-simVSSS/blob/master/images/cost.png "DQN-COST")

And the path plotted by the agent confirms the prediction.

![DQN-MAP](https://github.com/lucasbsimao/DQN-simVSSS/blob/master/images/init_map.png "DQN-MAP")
![DQN-FINAL](https://github.com/lucasbsimao/DQN-simVSSS/blob/master/images/final_map.png "DQN-FINAL")

## Implementation

The main modification in original simVSSS was the inclusion of the class ArtificialIntelligence.cpp. It is responsible for running the algorithm,
and it is called in Strategy.cpp.

```c++
void Strategy::runStrategy(vector<RobotStrategy*> robotStrategiesTeam, Map mapVision, bool followPolicy, bool saveNet){
    ...

    if(saveNet) artInt->saveNetwork();

    artInt->setFollowPolicy(followPolicy);

    ...
}
```

determine the terminal rewards in the same class

```c++
bool Strategy::calculateEnvReward(Map mapVision, int posX, int posZ){
    bool isTerminalState = false;

    if(mapVision.at(posX).at(posZ) == 8){
        this->reward = artInt->getMinReward();
    }else if(mapVision.at(posX).at(posZ) == 5){
        this->reward = artInt->getWallReward();
    }else if(mapVision.at(posX).at(posZ) == 1 || mapVision.at(posX).at(posZ) == 0){
        this->reward = artInt->getStdReward();
    }else if(mapVision.at(posX).at(posZ) == 2){
        this->reward = artInt->getMaxReward();
        isTerminalState = true;
    }else{
        cout << "Verify reward table! It is not returning any state reward." << endl;
    }

    if(artInt->getTakenActions() > 150){
        isTerminalState = true;
        this->reward = artInt->getStuckReward();
    }


    return isTerminalState;
}
```

and returns the next target

```c++
void Strategy::calculateNextTarget(RobotStrategy* robotStrategy){
    int agentX = robotStrategy->getPosition().getX()/SCALE_MAP;
    int agentZ = robotStrategy->getPosition().getZ()/SCALE_MAP;

    mapVision.at(agentX).at(agentZ) = 1; 

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

        actAimTerminal = calculateEnvReward(mapVision, futStateX, futStateZ);
    
        if(!actAimTerminal || (actAimTerminal && artInt->getTakenActions() > 150)){
            if(mapVision.at(futStateX).at(futStateZ) == 8){
                futStateX = agentX;
                futStateZ = agentZ;
            }
            if( mapVision.at(futStateX).at(futStateZ) == 5){
                futStateX = agentX;
                futStateZ = agentZ;
            }
        }

        reachingPoint = true;
    }
    
    robotStrategy->setTargetPosition(btVector3((futStateX+0.5)*SCALE_MAP, 0, (futStateZ+0.5)*SCALE_MAP));
}
```