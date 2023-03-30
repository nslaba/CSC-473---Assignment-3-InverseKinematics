#pragma once

#include "BetaSolver.h"
#include "EndEffectorWorldCoord.h"
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
	double t = 0.001;

	/* VARIABLES FOR THE ENTIRE IKSIM*/
	Eigen::VectorXd dtThetas;
	Eigen::VectorXd ErrorDtThetas;
	int cPointID = 0;
	glm::dvec3 dtX;
	Eigen::Vector3d dtX_eigen;
	// End effector in world coordinates AKA Pinitial
	Eigen::VectorXd P_endEffector;
	// P target in world coordinates
	Eigen::VectorXd P_target;
	// The actual end effector in reality
	Eigen::Vector4d endEffector;
	Eigen::Vector4d ErrorEndEffector;
	// Error
	double Error = 0.1;

	/* VARIABLES NEEDED FOR GETTING TO THE START OF THE SPLINE */
	float lerping_to_beginning_of_spline = 0.01;
	// the intermediate lerping end point
	Eigen::VectorXd end;

};