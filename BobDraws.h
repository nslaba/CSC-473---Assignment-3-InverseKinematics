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
#include <glm/gtc/matrix_transform.hpp>
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
	glm::mat4x4 identity{ 1.0 };
	glm::mat4x4 transform;
	glm::mat4x4 translateL1{1.0, 0.0, 0.0, 0.0,
							0.0, 1.0, 0.0, 0.0,
							0.0, 0.0, 1.0, 0.0,
							0.0, -m_bob->L1, 0.0, 1.0

	};
	glm::mat4x4 translateL2L3{ 1.0, 0.0, 0.0, 0.0,
							0.0, 1.0, 0.0, 0.0,
							0.0, 0.0, 1.0, 0.0,
							0.0, -(m_bob->L2 + m_bob->L3), 0.0, 1.0

	};

	void initializePs();
	Eigen::VectorXd dtThetas;
	int cPointID = 0;
	glm::dvec3 dtX;
	ControlPoint Ptarget;
	ControlPoint P; // Initially it is the starting of the spline
	double t = 0.001;
};