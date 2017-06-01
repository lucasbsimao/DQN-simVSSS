#include "Header.h"
#include "Physics.h"
#include "RobotStrategy.h"
#include "RobotPhysics.h"
#include "strategies/Strategy.h"

//Variables to determine how sensitive is the motion
#define SENS_ROT 5.0
#define SENS_OBS 1
#define SENS_TRANSL 30.0
#define FIRST_PERSON_CAMERA

class Scenario{

private:
    int x_ini = 0, y_ini = 0, z_ini = 0, but = -1;	//Glut - Initial x,y,z
	float aspect;					//Glut - Reason between width and height
	float initAngle = 60.0f;		//Glut - Initial angle
	float dx, dy, dz;				//Glut - Delta distance
	float ex = 1, ey = 1, ez = 1;	//Glut - Scale
	GLfloat rotX = 90.0,rotY = 90.0, rotZ = 0.0;	//Glut - Delta Rotation
	GLfloat rotX_ini, rotY_ini, rotZ_ini;		//Glut - Init rotation
	GLfloat obsX_ini, obsY_ini, obsZ_ini;		//Glut - Initial perspective observation position

	#ifndef FIRST_PERSON_CAMERA
		GLfloat obsX = 110, obsY = 0.0, obsZ = 75;	//Glut - Perspective observation
		float lookAt = 50;			//How far is the reference to the camera
	#else
		GLfloat obsX = (SIZE_WIDTH/2 + 1), obsY = 150.0, obsZ = SIZE_DEPTH/2;	//Glut - Perspective observation
		float lookAt = 0;
	#endif

	bool pauseScreen;				//Define if bullet is running
	bool quitStatus;
	bool singleStep;                //Define a single step in World
	float depthObs = 3000;			//How depth is the scenario

	bool followPolicy;
	bool saveNet;

	float timeLastLoop;				//Time in the last loop of glut
	float timeStep;					//Timestep of the Bullet

	int debugMode;					//Defines the debug style
    int numTeams;
    int numRobotsTeam;

	Mat image;
	Mat cvMapVision;

	vector<ModelStrategy*> strategies;
	vector<BulletObject*> gBodies;
	vector<RobotPhysics*> gRobots;
	Physics* physics;

	void renderRobotWheel(int i, RobotPhysics* rbt);
	void drawPhysicObjects();
	void debugWorldDrawer(int debugMode);

	void material(Color color[]);

	void renderFloor(BulletObject* btObj);
	void renderBall(BulletObject* btObj);
	void renderRobot(BulletObject* btObj);
	void renderBox(btRigidBody* box, Color clrUp[],Color clrBody[]);
	void renderRobot(RobotPhysics* rbt);

	void renderRobotDebugPoints();
	void drawRobotTarget();

	btVector3 calcAbsolutePosition(btVector3 relPos, ModelStrategy* strategy);

	//Variáveis de teste -------------------------------------------
	int graphLoop;

public:

	Scenario(Physics* physics ,vector<ModelStrategy*> strategies);
	~Scenario();

	//Auxiliar methods to run glut in classes
	void displayEvent(void);
	void reshapeEvent(GLsizei w, GLsizei h);
	void specialEvent(int tecla, int x, int y);
	void keyboardEvent(unsigned char tecla, int x, int y);
	void mouseEvent(int button, int state, int x, int y);
	void motionEvent(int x, int y);
	void timerEvent();
	void initLight();

	float getTimeStep() { return timeStep; }
	int getDebugMode() { return debugMode; }
	bool getQuitStatus() { return quitStatus; }
	bool getSingleStep();

	bool getFollowPolicy() { return followPolicy; }
	bool getSaveNetwork() { return saveNet; }

	void setSingleStep(bool singleStep);

	void updatePhysics(Physics* physics);
	Mat getSceneOpenCV();
};

class HandleGraphics{
private:
    static int argc;
    static char *argv[];
	static Scenario* scenario;
public:
    static void initGraphics(Physics* physics ,vector<ModelStrategy*> strategies, int argc, char *argv[]);
	//Static methods to run bullet in a class

	static void runGraphics(); //Initialize the simulator
	static void displayHandler();
	static void reshapeHandler(GLsizei w, GLsizei h);
	static void specialHandler(int tecla, int x, int y);
	static void keyboardHandler(unsigned char tecla, int x, int y);
	static void mouseHandler(int button, int state, int x, int y);
	static void motionHandler(int x, int y);
	static void timerHandler(int v);
	static Scenario* getScenario() { return scenario; }
};
