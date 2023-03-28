#pragma once

#include "BetaSolver.h"
#include "Transformations.h"
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
	Eigen::VectorXd dtThetas;
	int cPointID = 0;

	ControlPoint Ptarget;
	ControlPoint P; // Initially it is the starting of the spline
	double t = 0.001;

	glm::dvec3 dtX;
	Eigen::VectorXd dtX_eigen;
	Eigen::VectorXd endEffector;

	// TEMP EIGEN MATRIX VARIABLES FOR DEBUGGING
	Eigen::MatrixXd xRoll;
	Eigen::MatrixXd yRoll;
	Eigen::MatrixXd zRoll;
	Eigen::MatrixXd tL1;
	Eigen::MatrixXd tL2;
	Eigen::MatrixXd tL3;
	Eigen::MatrixXd transform;

};