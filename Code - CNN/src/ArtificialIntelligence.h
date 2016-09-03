#ifndef ARTIFICIAL_INTELIGENCE_H_
#define ARTIFICIAL_INTELIGENCE_H_

#define EPSILON 0.009

#include "Header.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "tiny_cnn/tiny_cnn.h"
#include "tiny_cnn/util/util.h"

using namespace tiny_cnn;
using namespace tiny_cnn::activation;

using namespace cv;

struct GameMemory{
    vec_t oldMapVision;
    int action;
    float reward;
    vec_t newMapVision;

    GameMemory(vec_t mv, int ac, float r, vec_t nMV):oldMapVision(mv),action(ac),reward(r),newMapVision(nMV){

    }
};

class ArtificialIntelligence{
private: 
    int gameDepth;
    int gameWidth;

    int countQ;

    Mat mapVision;
    vec_t imageInput;
    vec_t prevMapVision;
    int generationsRL;

    Map prevMDP;
    Map mdp;
    vector< vector<string> > policy;
    float accuracy;
    float noise;
    float discount;
    float actReward;
    float stdReward;

    float impartialState;
    float maxReward;
    float minReward;


    network<sequential> net;

    int countBuffer;

    int actualAction;
    bool isTerminal;

    bool followPolicy;

    float epsilon;
    int batch;
    float gamma;
    int buffer;

    int epoch;
    float numEpochs;
    int takenAct;

    vector<GameMemory> replay; 

    
    int calculateActualState(vec_t output);
    float calculateQValue();

    bool reachDeepState();

    vec_t processImage();
    void processRL();
    int calculateMaxOutput(vec_t output);

    vector<GameMemory> selectRandomBatch();

    void showBestResult();

    void constructNet();

public:
	ArtificialIntelligence();

    Map getMdp() {return mdp;}
    int getActualAction() { return actualAction; }
    void setMap(Mat mapVision, float reward, bool isTerminal);

    int getEpoch() { return epoch; }
    int getTakenActions() { return takenAct; }

    bool calculateMDPStates();
    void reinitMapProp();

    void saveNetwork();

    float getMaxReward() { return maxReward; }
    float getImpartialState() { return impartialState; }
    float getMinReward() { return minReward; }
    float getStdReward() { return stdReward; }

    void setFollowPolicy(bool followPolicy);
};

#endif