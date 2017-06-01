#ifndef SIR_H_
#define SIR_H_

#include "Header.h"

typedef vector<vector<float> > Map;

//Standard colors of the simulator
struct Color{
	float r;
	float g;
	float b;

	Color(){}
	Color(float r0, float g0, float b0):r(r0),g(g0),b(b0){}
};

//All rigid bodies are searched by the vector of BulletObject
struct BulletObject{
    int id;
    string name;
    Color clr;
    bool hit;
    btRigidBody* body;
    btVector3 halfExt; //Used only for compound shapes
    BulletObject(btRigidBody* b,string n,Color clr0) : name(n),body(b),clr(clr0),id(-1),hit(false),halfExt(btVector3(0,0,0)) {}
};

#endif
