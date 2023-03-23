#pragma once

#include "Jacobian.h"
#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "Bob.h"
#include "BaseSimulator.h"
#include <glm/glm.hpp>
#include <glm/geometric.hpp>
#include "HermiteSpline.h"
#include <math.h>
#include "BetaSolver.h"


class BobDraws :
	public BaseSimulator
{
public:
	BobDraws(const std::string& name, HermiteSpline* drawing, Bob* bob);
	~BobDraws();
	int step(double time);
	int command(int argc, myCONST_SPEC char** argv);
	int init(double time) { return 0; };
private:
	HermiteSpline* drawingPath;
	Bob* m_bob;
	void initializePs();
	int cPointID = 0;
	glm::dvec3 dtX;
	ControlPoint Ptarget;
	ControlPoint P; // Initially it is the starting of the spline
	double t = 0.001;
};