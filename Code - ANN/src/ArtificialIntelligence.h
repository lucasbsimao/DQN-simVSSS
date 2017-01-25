#ifndef ARTIFICIAL_INTELIGENCE_H_
#define ARTIFICIAL_INTELIGENCE_H_

#define EPSILON 0.009

#include "Header.h"

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "tiny_dnn/tiny_dnn.h"
#include "tiny_dnn/util/util.h"

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <assert.h>
#include <random>


using namespace tiny_dnn;
using namespace tiny_dnn::activation;

using namespace cv;

class Neuron;

typedef vector<Neuron> Layer;

struct ActivationFunc{
	double virtual activation(double z) = 0;
	double virtual derivateActivation(double z) = 0;
};

struct Sigmoid: public ActivationFunc{
	double activation(double z){
		double activation = 1/(1+exp(-z));
		return activation;
	}

	double derivateActivation(double z){
		double derivate = z*(1-z);
		return derivate;
	}
};

struct LeakyRelu: public ActivationFunc{
	double activation(double z){
		if(z > 0)
			return z;
		else
			return 0.01*z;
	}

	double derivateActivation(double z){
		if(z > 0)
			return 1;
		else
			return 0.01;
	}
};

struct Identity: public ActivationFunc{
	double activation(double z){
		return z;
	}

	double derivateActivation(double z){
		return 1;
	}
};

struct Connection{
	double outWeight;
	double weightGrad;
	double totalDelta;
};

class Neuron{
public:
	Neuron(int numOutputs, int nodeId, ActivationFunc* function);
	void feedForward(Layer &prevLayer);
	ActivationFunc* activationFunc;
	double sigmoid(double z);
	double derivateSigmoid(double targetVal);
	double leakyRelu(double z);
	double derivateLeakyRelu(double targetVal);
	double identity(double z);
	double derivateIdentity();
	void setOutputValue(double val);
	double getOutputValue() { return m_output;};
	vector<Connection> getConnections() { return m_connections; };
	void setWeights(vector<double> outWeigths);
	double getDelta() {return m_delta;};

	void calcOutputDelta(double targetVal);
	void calcHiddenDelta(Layer& nextLayer);

	void updateDeltaWeights(Layer& prevLayer);
	void update(int m);
	void setIsBias(bool isBias);
private:
	bool isBias;
	int m_nodeId;
	double m_delta;
	double randomWeight(){return rand()/double(RAND_MAX);};
	double m_output;
	double m_hypOut;
	vector<Connection> m_connections;
};

class NeuralNetwork{
public:
    NeuralNetwork(){};
	NeuralNetwork(vector<int> topology, vector<ActivationFunc*> activations);
	void feedForward(vector<double> inputVals);
	void backProp(vector<double> targetVals);
	void SGD(int miniBatch);
	vector<double> getResults();
	double getError() {return m_error; };

	void calcActualError(vector<double> targetVals);
private:
	vector<Layer> m_layers;
	double m_error;
};

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

    Map mapVision;
    vec_t imageInput;
    vec_t prevMapVision;
	vec_t concatPrevMapVisions;
	vec_t concatInputMapVisions;
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
	float wallReward;
	float stuckReward;

    NeuralNetwork myNet;

    network<sequential> net;
	RMSprop mptimizer;

	vec_t imageStd;

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

	vector<float> listErrors;
	vector<float> listPolicies;
	float policyValue;
	int numCopies;

	int nShowError;

    vector<GameMemory> replay; 
    
    int calculateActualState(vec_t output);
    float calculateQValue();

    bool reachDeepState();

    vec_t processImage();
    void processRL();
    int calculateMaxOutput(vec_t output);
	void showState(vec_t vision);

    vector<GameMemory> selectRandomBatch();

    void showBestResult();

    void constructNet();
	void initNetwork();

public:
	ArtificialIntelligence();

    Map getMdp() {return mdp;}
    int getActualAction() { return actualAction; }
    void setMap(Map mapVision, float reward, bool isTerminal);

    int getEpoch() { return epoch; }
    int getTakenActions() { return takenAct; }

    bool calculateMDPStates();
    void reinitMapProp();

    void saveNetwork();

    float getMaxReward() { return maxReward; }
    float getImpartialState() { return impartialState; }
    float getMinReward() { return minReward; }
    float getStdReward() { return stdReward; }
	float getWallReward() { return wallReward; }
	float getStuckReward() { return stuckReward; }

    void setFollowPolicy(bool followPolicy);
};

#endif