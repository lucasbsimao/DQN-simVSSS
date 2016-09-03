#include "Scenario.h"

Scenario* HandleGraphics::scenario = NULL;
void HandleGraphics::initGraphics(Physics* physics ,vector<ModelStrategy*> strategies,int argc, char* argv[]){
    glutInit(&argc, argv);
    scenario = new Scenario(physics,strategies);
}

void HandleGraphics::runGraphics(){
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(50, 50);
    glutInitWindowPosition(900, 150);
    glutCreateWindow("Path Planning");
    
    glutDisplayFunc(displayHandler);
    glutReshapeFunc(reshapeHandler);
    glutKeyboardFunc(keyboardHandler);
    glutTimerFunc(5, timerHandler, 0);
    glutMouseFunc(mouseHandler);
    glutMotionFunc(motionHandler);
    glutSpecialFunc(specialHandler);
    
    if(scenario) scenario->initLight();

    glutMainLoop();
}

void HandleGraphics::displayHandler(){
    scenario->displayEvent();
}

void HandleGraphics::reshapeHandler(GLsizei w, GLsizei h){
    scenario->reshapeEvent(w,h);
}

void HandleGraphics::specialHandler(int tecla, int x, int y){
    scenario->specialEvent(tecla,x,y);
}

void HandleGraphics::keyboardHandler(unsigned char tecla, int x, int y){
    scenario->keyboardEvent(tecla,x,y);
}

void HandleGraphics::mouseHandler(int button, int state, int x, int y){
    scenario->mouseEvent(button,state,x,y);
}

void HandleGraphics::motionHandler(int x, int y){
    scenario->motionEvent(x,y);
}

void HandleGraphics::timerHandler(int v){
    scenario->timerEvent();

    glutTimerFunc(5, timerHandler, 0);
}

Scenario::Scenario(Physics* physics ,vector<ModelStrategy*> strategies){
    this->gRobots = physics->getAllRobots();
    this->gBodies = physics->getAllBtObj();
    this->physics = physics;
    this->strategies = strategies;
    singleStep = true;
    quitStatus = false;
    pauseScreen = true;

	followPolicy = false;
	saveNet = false;

	timeLastLoop = 0;

	debugMode = 0;

	image = Mat(50, 50,CV_8UC3);

	numTeams = strategies.size();
	if(numTeams == 0){
        quitStatus = true;
        cout << "You must set a strategy to run the simulator!\n";
        exit(0);
	}

	this->numRobotsTeam = NUM_ROBOTS_TEAM;

}

Scenario::~Scenario(){

}

void Scenario::updatePhysics(Physics* physics){
	this->gRobots = physics->getAllRobots();
    this->gBodies = physics->getAllBtObj();
}

void Scenario::initLight(){
	GLfloat luzAmbiente[4] = { 0.35, 0.35, 0.35, 1.0 };
	GLfloat luzDifusa[4] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat luzEspecular[4] = { 0.5, 0.5, 0.5, 1.0 };
	GLfloat posLuz[4] = { SIZE_WIDTH/2, 200, SIZE_DEPTH/2, 1.0 };

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, luzAmbiente);

	glLightfv(GL_LIGHT0, GL_AMBIENT, luzAmbiente);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, luzDifusa);
	glLightfv(GL_LIGHT0, GL_SPECULAR, luzEspecular);
	glLightfv(GL_LIGHT0, GL_POSITION, posLuz);

	glEnable(GL_LINE_SMOOTH);
	glShadeModel(GL_SMOOTH);

	glClearColor(0.6f, 0.6f, 0.6f, 0.0f);

	glEnable(GL_DEPTH_TEST);

	glEnable(GL_NORMALIZE);

	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

	glEnable(GL_LIGHTING);

	glEnable(GL_COLOR_MATERIAL);

	glEnable(GL_LIGHT0);


}

void Scenario::displayEvent(){

	if(debugMode == 0){
        glEnable(GL_LIGHTING);
        glClearColor(0.6f, 0.6f, 0.6f, 0.0f);
	} else{
        glDisable(GL_LIGHTING);
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	}

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -lookAt);
	glRotatef(rotX, 1, 0, 0);

	#ifndef FIRST_PERSON_CAMERA
		glColor3f(1,0,0);
		material(Color(0.78, 0.57, 0.11));
		glutSolidCube(0.5);
	#endif

	glRotatef(rotY, 0, 1, 0);
	glTranslatef(-obsX, -obsY, -obsZ);

    float timeNow = glutGet(GLUT_ELAPSED_TIME);
	timeStep = (float)(timeNow - timeLastLoop)/1000.;
	timeLastLoop = timeNow;
    //drawRobotTarget();

	if(!physics->isRefreshingWorld()){
		renderRobotDebugPoints();
		physics->startDebug();
		
		if(debugMode == 0)drawPhysicObjects();

		glPixelStorei(GL_PACK_ALIGNMENT, (image.step & 3) ? 1 : 4);
		
		glPixelStorei(GL_PACK_ROW_LENGTH, image.step/image.elemSize());
		
		glReadPixels(0, 0, image.cols, image.rows, GL_BGR, GL_UNSIGNED_BYTE, image.data);
		
		Mat flipped;
		cv::flip(image, flipped, 0);
		cvMapVision = flipped;
	}

	glutSwapBuffers();
}

Mat Scenario::getSceneOpenCV(){
	/*namedWindow( "Display window", WINDOW_AUTOSIZE );
	
	imshow("Display window", cvMapVision);
	waitKey(1);*/
	return cvMapVision;
}

void Scenario::setSingleStep(bool singleStep){
    this->singleStep = singleStep;
}

bool Scenario::getSingleStep(){
    if(!pauseScreen || singleStep)
        return true;
    else
        return false;
}

void Scenario::drawPhysicObjects(){
	for(int i=0;i<gBodies.size();i++)
    {
        if(gBodies[i]->body->getCollisionShape()->getShapeType()==STATIC_PLANE_PROXYTYPE)
            renderFloor(gBodies[i]);
        else if(gBodies[i]->body->getCollisionShape()->getShapeType()==SPHERE_SHAPE_PROXYTYPE)
            renderBall(gBodies[i]);
        else if(!gBodies[i]->name.compare("wall") || !gBodies[i]->name.compare("corner")){
            Color clrUp[] = {Color(0.3,0.3,0.3),Color(0.3,0.3,0.3)};
            Color clrBody[] = {Color(0.3,0.3,0.3),Color(0.3,0.3,0.3),Color(0.3,0.3,0.3)};
            renderBox(gBodies[i]->body,clrUp,clrBody);
        }
        gBodies[i]->hit=false;
    }

    for(int i=0;i<gRobots.size();i++){
    	renderRobot(gRobots[i]);
    }
}

void Scenario::reshapeEvent(GLsizei w, GLsizei h){

	if (h == 0)
		h = 1;

	glViewport(0, 0, w, h);
	aspect = (float)w / h;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(60, aspect, 0.1f, depthObs);

	glMatrixMode(GL_MODELVIEW);
}

void Scenario::keyboardEvent(unsigned char key, int x, int y){

	switch (key){
		case 27:
			quitStatus = true;

			exit(0);
			break;
		case 32:
			pauseScreen = !pauseScreen;
			break;
		case 'd': case 'D':{
			debugMode++;
			if(debugMode > 2) debugMode = 0;
			physics->setDebugWorld(debugMode);
		}break;
		case 'p': case 'P':{
			if(pauseScreen){
                singleStep = true;
			}
		}break;
		case 's': case 'S':{
                saveNet = !saveNet;
				string message = saveNet ? "SaveNet: True" : "SaveNet : False";
				cout << message << endl;
		}break;
		case 't': case 'T':{
                followPolicy = !followPolicy;
				string message = followPolicy ? "FollowPolicy: True" : "FollowPolicy: False";
				cout << followPolicy << endl; 
		}break;
	}

	glutPostRedisplay();

}

void Scenario::specialEvent(int tecla, int x, int y)
{
	switch (tecla)
	{
		case GLUT_KEY_UP:
		{
			float xrotrad, yrotrad;
			yrotrad = (rotY / 180 * PI);
			xrotrad = (rotX / 180 * PI);
			obsX += float(sin(yrotrad));
			obsY -= float(sin(xrotrad));
			obsZ -= float(cos(yrotrad));
		}break;

		case GLUT_KEY_DOWN:
		{
			float xrotrad, yrotrad;
			yrotrad = (rotY / 180 * PI);
			xrotrad = (rotX / 180 * PI);
			obsX -= float(sin(yrotrad));
			obsY += float(sin(xrotrad));
			obsZ += float(cos(yrotrad));
		}break;

		case GLUT_KEY_RIGHT:
		{
			float yrotrad;
			yrotrad = (rotY / 180 * PI);
			obsX += float(cos(yrotrad)) * 0.2;
			obsZ += float(sin(yrotrad)) * 0.2;
		}break;

		case GLUT_KEY_LEFT:
		{
			float yrotrad;
			yrotrad = (rotY/ 180 * PI);
			obsX -= float(cos(yrotrad)) * 0.2;
			obsZ -= float(sin(yrotrad)) * 0.2;
		}break;
	}

	glutPostRedisplay();
}

void Scenario::mouseEvent(int button, int state, int x, int y){
	if (state == GLUT_DOWN){
		but = button;
		x_ini = x;
		y_ini = y;
		rotX_ini = rotX;
		rotY_ini = rotY;
	}
	else{
		but = -1;
	}
}

void Scenario::motionEvent(int x, int y){

	if(but == GLUT_LEFT_BUTTON){
	    int diffx=x_ini - x; //check the difference between the current x and the last x position
		int diffy=y_ini - y; //check the difference between the current y and the last y position

	    rotX = ((float)rotX_ini - (float)diffy); //set the xrot to xrot with the additionof the difference in the y position
	    rotY = ((float)rotY_ini - (float)diffx);    //set the xrot to yrot with the additionof the difference in the x position
	}

	glutPostRedisplay();
}

void Scenario::renderRobotDebugPoints(){
	for(int i = 0; i < numTeams;i++){
        for(int j = 0; j < numRobotsTeam;j++){
    		if(strategies[i]->getDrawComponents().size() > 0){
    			vector<DrawComponents> drawComp = strategies[i]->getDrawComponents();
                //btVector3 unitTarget = targetPos - robotPos;
                for(int w = 0; w < drawComp.size(); w++){
					vector<btVector3> listPoints = drawComp.at(w).drawPoints;

					Color clrs[] = {drawComp.at(w).color,drawComp.at(w).color,drawComp.at(w).color};
					material(clrs);
					glPushMatrix();
						glBegin(GL_QUADS);
						for(int k = 0; k < listPoints.size(); k ++){
							glVertex3f(listPoints.at(k).getX(),listPoints.at(k).getY(),listPoints.at(k).getZ());
						}
						glEnd();
					glPopMatrix();
                } 
    		}
        }
    }
}

void Scenario::drawRobotTarget(){
    Color clr2[] = {Color(1,1,1),Color(1,1,1),Color(1,1,1)};
    Color clr1[] = {Color(0,1,0),Color(0,1,0),Color(0,1,0)};

    for(int i = 0; i < numTeams;i++){
        for(int j = 0; j < numRobotsTeam;j++){
    		if(strategies[i]->getRobotStrategiesTeam().size() > 0){
    			btVector3 relTargetPos = strategies[i]->getRobotStrategiesTeam()[j]->getTargetPosition();
                btVector3 targetPos = calcAbsolutePosition(relTargetPos,strategies[i]);
                btVector3 relRobotPos = strategies[i]->getRobotStrategiesTeam()[j]->getPosition();
                btVector3 robotPos = calcAbsolutePosition(relRobotPos,strategies[i]);
                //btVector3 unitTarget = targetPos - robotPos;
                if(i == 0)material(clr1);
                if(i == 1)material(clr2);
                glPushMatrix();
                   //glTranslatef(robotPos.getX(),4,robotPos.getZ());
                    glBegin(GL_LINES);
                        glVertex3f(robotPos.getX(),4,robotPos.getZ());
                        glVertex3f(targetPos.getX(),4,targetPos.getZ());
                    glEnd();
                glPopMatrix();
    		}
        }
    }
}

void Scenario::timerEvent(){

	glutPostRedisplay();
}

void Scenario::material(Color color[]){
    GLfloat diffuseColor[] = { color[0].r, color[0].g, color[0].b, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseColor);
    GLfloat ambientColor[] = {color[1].r, color[1].g, color[1].b, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambientColor);
    GLfloat specularColor[] = { color[2].r, color[2].g, color[2].b, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specularColor);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 10.0);
}

void Scenario::renderBall(BulletObject* btObj)
{
	GLUquadricObj* quad = gluNewQuadric();
	Color clr[] = {Color(1, 0.5, 0.0),Color(1, 0.5, 0.0),Color(1, 0.5, 0.0)};
	material(clr);
    btRigidBody* ball=btObj->body;

    float r=((btSphereShape*)ball->getCollisionShape())->getRadius();
    btTransform t;

    ball->getMotionState()->getWorldTransform(t);
    float mat[16];

    t.getOpenGLMatrix(mat);
    glPushMatrix();
        glMultMatrixf(mat);     //translation,rotation

        gluSphere(quad,r,20,20);
    glPopMatrix();
    gluDeleteQuadric(quad);
}

void Scenario::renderFloor(BulletObject* btObj)
{

    btRigidBody* plane=btObj->body;

    btTransform t;
    plane->getMotionState()->getWorldTransform(t);
    float mat[16];
    t.getOpenGLMatrix(mat);
    Color clr[]{Color(0.2, 0.2, 0.2),Color(0.2, 0.2, 0.2),Color(0.2, 0.2, 0.2)};
    material(clr);
	glPushMatrix();
		glMultMatrixf(mat);
		glBegin(GL_QUADS);
			glVertex3f(-10,0.0,SIZE_DEPTH+10);
			glVertex3f(-10,0.0,-10);
			glVertex3f(SIZE_WIDTH+10,0.0,-10);
			glVertex3f(SIZE_WIDTH+10,0.0,SIZE_DEPTH+10);
		glEnd();
	glPopMatrix();
}

void Scenario::renderRobot(RobotPhysics* rbt)
{
    glDisable(GL_RESCALE_NORMAL);
	btTransform trans;
	float m[16];


	// draw wheels

	for(int i = 0; i < rbt->getRaycast()->getNumWheels();i++){
		renderRobotWheel(i,rbt);
	}

	Color clr[] = {Color(0.5, 0.5, 0.5),Color(0.5, 0.5, 0.5),Color(0.5, 0.5, 0.5)};
    Color clrUp[] = {rbt->getColorTeam(),rbt->getColorPlayer()};

    renderBox(rbt->getRigidBody(),clrUp,clr);

	btVector3 unitVecX = rbt->getLocalUnitVecX();
	btVector3 unitVecZ = rbt->getLocalUnitVecZ();
	btVector3 posVec = rbt->getPosition();

	glPushMatrix();
		glTranslatef(posVec.getX(),8,posVec.getZ());
		Color clrF[] = {Color(0, 0, 1),Color(0, 0, 1),Color(0, 0, 1)};
		material(clrF);
		glBegin(GL_LINES);
			glVertex3f(0,0,0);
			glVertex3f(unitVecX.getX()*10,0,unitVecX.getZ()*10);
		glEnd();

        Color clrS[] = {Color(0, 1, 0),Color(0, 1, 0),Color(0, 1, 0)};
		material(clrF);
		glBegin(GL_LINES);
			glVertex3f(0,0,0);
			glVertex3f(unitVecZ.getX()*10,0,unitVecZ.getZ()*10);
		glEnd();
	glPopMatrix();

	glEnable(GL_RESCALE_NORMAL);
}

void Scenario::renderRobotWheel(int i, RobotPhysics* rbt){
	float wheelWidth = rbt->getWheelWidth();
	float wheelSteeringRd = rbt->getSteeringWheelRadius();
	float wheelFreeRd = rbt->getFreeWheelRadius();

    Color clr[]={Color(0.8, 0.8, 0.8),Color(0.8, 0.8, 0.8),Color(0.8, 0.8, 0.8)};

	float m[16];
	GLUquadricObj *quadObj = gluNewQuadric();
	gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
	gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);

	//synchronize the wheels with the (interpolated) chassis worldtransform
	rbt->getRaycast()->updateWheelTransform(i,true);
	//draw wheels (cylinders)
	rbt->getRaycast()->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);

	glPushMatrix();
        material(clr);
		glMultMatrixf(m);
		glRotatef(90.0, 0.0, 1.0, 0.0);

		glPushMatrix();
			glTranslatef(0.0, 0.0, wheelWidth/2);

			if(i>=2) gluDisk(quadObj,0,wheelSteeringRd,15, 10);

			gluQuadricTexture(quadObj, true);
			glTranslatef(0.0, 0.0, -wheelWidth);

			if(i>=2) gluCylinder(quadObj, wheelSteeringRd, wheelSteeringRd, wheelWidth, 15, 10);

			gluQuadricTexture(quadObj, false);

			glRotatef(-180.0, 0.0, 1.0, 0.0);

			if(i>=2) gluDisk(quadObj,0,wheelSteeringRd,15, 10);
			else glutSolidSphere(wheelFreeRd,15, 10);

		glPopMatrix();
	glPopMatrix();
	gluDeleteQuadric(quadObj);
}

void Scenario::renderBox(btRigidBody* box, Color clrUp[],Color clrBody[])
{
    btBoxShape* boxShape = (btBoxShape*)box->getCollisionShape();
    btVector3 extent=boxShape->getHalfExtentsWithoutMargin();

	btTransform t;
	box->getMotionState()->getWorldTransform(t);
	float mat[16];
	t.getOpenGLMatrix(mat);
	glPushMatrix();
        BulletObject* btObj = (BulletObject*)box->getUserPointer();
        if(btObj->name == "robot"){
            extent = btObj->halfExt;
            glTranslatef(0,4,0);
        }
        Color clr[] = {clrBody[0],clrBody[1],clrBody[2]};
        material(clr);
        glMultMatrixf(mat);
        glBegin(GL_QUADS);
            glVertex3f(-extent.x(),extent.y(),-extent.z());
            glVertex3f(-extent.x(),-extent.y(),-extent.z());
            glVertex3f(-extent.x(),-extent.y(),extent.z());
            glVertex3f(-extent.x(),extent.y(),extent.z());
        glEnd();
        glBegin(GL_QUADS);
            glVertex3f(extent.x(),extent.y(),-extent.z());
            glVertex3f(extent.x(),-extent.y(),-extent.z());
            glVertex3f(extent.x(),-extent.y(),extent.z());
            glVertex3f(extent.x(),extent.y(),extent.z());
        glEnd();
        glBegin(GL_QUADS);
            glVertex3f(-extent.x(),extent.y(),extent.z());
            glVertex3f(-extent.x(),-extent.y(),extent.z());
            glVertex3f(extent.x(),-extent.y(),extent.z());
            glVertex3f(extent.x(),extent.y(),extent.z());
        glEnd();
        glBegin(GL_QUADS);
            glVertex3f(-extent.x(),extent.y(),-extent.z());
            glVertex3f(-extent.x(),-extent.y(),-extent.z());
            glVertex3f(extent.x(),-extent.y(),-extent.z());
            glVertex3f(extent.x(),extent.y(),-extent.z());
        glEnd();
        glBegin(GL_QUADS);
            glVertex3f(-extent.x(),-extent.y(),-extent.z());
            glVertex3f(-extent.x(),-extent.y(),extent.z());
            glVertex3f(extent.x(),-extent.y(),extent.z());
            glVertex3f(extent.x(),-extent.y(),-extent.z());
        glEnd();
        glBegin(GL_QUADS);
            glVertex3f(-extent.x(),extent.y(),-extent.z());
            glVertex3f(-extent.x(),extent.y(),0);
            glVertex3f(0,extent.y(),0);
            glVertex3f(0,extent.y(),-extent.z());
        glEnd();
        glBegin(GL_QUADS);
            glVertex3f(0,extent.y(),0);
            glVertex3f(0,extent.y(),extent.z());
            glVertex3f(extent.x(),extent.y(),extent.z());
            glVertex3f(extent.x(),extent.y(),0);
        glEnd();
        Color clrUpPlr[] = {clrUp[0],clrUp[0],clrUp[0]};
        material(clrUpPlr);
        glBegin(GL_QUADS);
            glVertex3f(extent.x(),extent.y(),-extent.z());
            glVertex3f(extent.x(),extent.y(),0);
            glVertex3f(0,extent.y(),0);
            glVertex3f(0,extent.y(),-extent.z());
        glEnd();
        Color clrUpTeam[] = {clrUp[1],clrUp[1],clrUp[1]};
        material(clrUpTeam);
        glBegin(GL_QUADS);
            glVertex3f(-extent.x(),extent.y(),extent.z());
            glVertex3f(-extent.x(),extent.y(),0);
            glVertex3f(0,extent.y(),0);
            glVertex3f(0,extent.y(),extent.z());
        glEnd();
	glPopMatrix();
	glPopMatrix();
}

btVector3 Scenario::calcAbsolutePosition(btVector3 relPos, ModelStrategy* strategy){
    float absX = relPos.getX();
    float absZ = relPos.getZ();
    if(strategy->getAttackDir() == -1){
        absX = SIZE_WIDTH - relPos.getX();
        absZ = SIZE_DEPTH - relPos.getZ();
    }

    return btVector3(absX,0,absZ);
}
