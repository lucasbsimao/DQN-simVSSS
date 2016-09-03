#include "ArtificialIntelligence.h"

ArtificialIntelligence::ArtificialIntelligence(){
	accuracy = 0.8;
	noise = 0.1;
	discount = 1.0;
	stdReward = -0.025;
	generationsRL = 0;

	impartialState = -15;
	maxReward = 1;
	minReward = -0.25;
	countQ = 0;

	//DQN constants

	epoch = 0;

	actualAction = -1;

	epsilon = 1.0f;
	gamma = 0.98;

	batch = 20;
	buffer = 210;
	countBuffer = 0;
	isTerminal = false;

	followPolicy = false;

	numEpochs = 3000;
	actReward = 0;

	constructNet();
	std::ifstream input("net_backup.txt");
	net.load(input);
}

void ArtificialIntelligence::reinitMapProp(){
	takenAct = 0;
}

void ArtificialIntelligence::setFollowPolicy(bool followPolicy){
	this->followPolicy = followPolicy;
}

void ArtificialIntelligence::saveNetwork(){
	std::ofstream file("net_ostream.txt");
	file << net;

	std::ofstream file2("net_backup.txt");
	net.save(file2);
}

void ArtificialIntelligence::constructNet() {

	typedef convolutional_layer<activation::identity> conv;
    typedef max_pooling_layer<relu> pool;

    const int n_fmaps = 32; ///< number of feature maps for upper layer
    const int n_fmaps2 = 64; ///< number of feature maps for lower layer
    const int n_fc = 64; ///< number of hidden units in fully-connected layer

    net << convolutional_layer<leaky_relu>(50, 50, 6, 1, 32, padding::valid, true, 4,4)
    << convolutional_layer<leaky_relu>(12, 12, 4, 32, 64)
    << convolutional_layer<leaky_relu>(9, 9, 3, 64, 64)
	<< fully_connected_layer<leaky_relu>(7*7*64, 512)
    << fully_connected_layer<identity>(512, 4);

	net.init_weight();
}

void ArtificialIntelligence::setMap(Mat mapVision, float reward, bool isTerminal){
	cout << "EPOCH: " << epoch << endl;
	Mat im_gray;
	cvtColor(mapVision,im_gray,CV_RGB2GRAY);

	if(takenAct > 0){
		this->isTerminal = isTerminal;
		this->prevMapVision = vec_t(this->imageInput);
		actReward = reward;
	}

	//Use to increase window and see what is happening
	/*Mat imVision;
	cv::resize(mapVision,imVision,cv::Size(150,150));
	namedWindow( "Display window", WINDOW_AUTOSIZE );
	
	imshow("Display window", imVision);
	waitKey(1);*/
	this->mapVision = im_gray;
	
	processRL();

	epoch++;
	takenAct++;
}

vector<GameMemory> ArtificialIntelligence::selectRandomBatch(){
	vector<GameMemory> batchSelect;

	for(int i = 0; i < batch; i++){
		int index = rand()%buffer;
		batchSelect.push_back(replay.at(index));
	}

	return batchSelect;
}

void ArtificialIntelligence::processRL(){
	imageInput = processImage();

	vec_t output = net.predict(imageInput);

cout << "{ " << output.at(0) << ", " << output.at(1) << ", " << output.at(2) <<  ", " << output.at(3) << " }" << endl;


	if(followPolicy){
		actualAction = calculateMaxOutput(output);
		return;
	}
	
	int prevAction = actualAction;
	actualAction = calculateActualState(output);

	if(takenAct == 0) return;
	cout << "Reward: " << actReward << endl;

	GameMemory gameMemory(prevMapVision,prevAction, actReward, imageInput);

	if(replay.size() < buffer){
		replay.push_back(gameMemory);
	}else{
		
		replay.at(countBuffer) = gameMemory;

		vector<GameMemory> miniBatch = selectRandomBatch();

		cout << "passou 3" << endl;

		vector<vec_t> inputs_train;
		vector<vec_t> outputs_train;
		for(int i = 0; i < batch; i++){
			GameMemory mem = miniBatch.at(i);

			//cout << "index 1: " << i << " " << mem.oldMapVision.size() << " " << mem.newMapVision.size() << endl;
			vec_t oldQ = net.predict(mem.oldMapVision);
			vec_t newQ = net.predict(mem.newMapVision);
			float maxQ = calculateMaxOutput(newQ);

			//cout << "index 2: " << i << endl;

			vec_t oldOutputs(oldQ);

			float stateValue = actReward + gamma*maxQ;
			if(isTerminal)
				stateValue = actReward;
			
			oldOutputs.at(mem.action) = stateValue;

			inputs_train.push_back(mem.oldMapVision);
			outputs_train.push_back(oldOutputs);
		}

		cout << "passou 4" << endl;

		//cout << "passou 3" << endl;

		RMSprop optimizer;
		net.fit<mse>(optimizer,inputs_train, outputs_train,batch,1);
		//cout << "passou 4" << endl;

		if(countBuffer < buffer-1)
			countBuffer++;
		else
			countBuffer = 0;

		epsilon -= 1.0f/numEpochs;
		cout << "ep: " << epsilon << endl;
		//if(epoch == 1000){
			//std::ofstream output("nets.txt");
			//output << net;
		//}
	}
}

vec_t ArtificialIntelligence::processImage(){
	vec_t inputNet;
	Mat normImage = mapVision.clone();
	
	cv::Mat_<uint8_t> resized;
    cv::resize(normImage, resized, cv::Size(50, 50));

	float scale = 1.f/(255.f);
    std::transform(resized.begin(), resized.end(), std::back_inserter(inputNet),
                   [=](uint8_t c) { return c * scale; });

	/*for(int i = 0; i < inputNet.size(); i ++){
		cout << inputNet.at(i) << endl;
	}*/

	return inputNet;
}

bool ArtificialIntelligence::reachDeepState(){


	return true;
}

int ArtificialIntelligence::calculateMaxOutput(vec_t output){
	float maxAction = -10000;
	int action;
	for(int i = 0; i < output.size();i++){
		if(output.at(i) > maxAction){
			maxAction = output.at(i);
			action = i;
		}
	}

	return action;
}

int ArtificialIntelligence::calculateActualState(vec_t output){
	int action = calculateMaxOutput(output);

	float randF = (rand()%100)/100.f;

	if(randF < epsilon)
		action = rand()%(4);

	return action;
}

float ArtificialIntelligence::calculateQValue(){
	return 0;
}

void ArtificialIntelligence::showBestResult(){
	//for(int j = gameDepth-1; j >= 0;j--){
	for(int j = 0; j < gameDepth;j++){
		for(int i = 0; i < gameWidth ;i++){
			printf("%.2f\t",mdp.at(j).at(i));
		}
		cout << endl;
	}

	cout << endl << "Policy:" << endl;
	//for(int j = gameDepth-1; j >= 0;j--){
	for(int j = 0; j < gameDepth;j++){
		for(int i = 0; i < gameWidth ;i++){
			cout << policy.at(j).at(i) << "\t";
		}
		cout << endl;
	}
}
