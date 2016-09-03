#ifndef GL_DEBUG_DRAWER_H
#define GL_DEBUG_DRAWER_H

#include "LinearMath/btIDebugDraw.h"
#include "GL/glut.h"
#include <iostream>

using namespace std;

class GLDebugDrawer : public btIDebugDraw
{
	vector<int> m_debugMode;
	bool drawScenario;

public:

	GLDebugDrawer();
	virtual ~GLDebugDrawer();

	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& fromColor, const btVector3& toColor);

	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color);

	virtual void	drawSphere (const btVector3& p, btScalar radius, const btVector3& color);

	virtual void	drawTriangle(const btVector3& a,const btVector3& b,const btVector3& c,const btVector3& color,btScalar alpha);

	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color);

	virtual void	reportErrorWarning(const char* warningString);

	virtual void	draw3dText(const btVector3& location,const char* textString);

	virtual void	setDebugMode(vector<int> debugMode);

	virtual vector<int>	getDebugMode() const;

	virtual void    setDrawScenarioMode(bool drawScenario);

	virtual bool    getDrawScenarioMode() { return drawScenario; }

};

#endif//GL_DEBUG_DRAWER_H
