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
	void lerp(Eigen::Vector4d start, Eigen::Vector4d end, float scalar);
	void updateAngles(Eigen::VectorXd newThetas);
	//FOR TESTING
	void print_vals_for_testing(Eigen::Vector4d step);

private:
	HermiteSpline* drawingPath;
	Bob* m_bob;
	
	void initializePs();
	double t = 0.001;

	/* VARIABLES FOR THE ENTIRE IKSIM*/
	
	// Storage for dtThetas
	Eigen::VectorXd dtThetas;
	// Control Point ID
	int cPointID = 0;
	// End effector in world coordinates AKA Pinitial (where the arm actually is)
	Eigen::VectorXd P_endEffector;
	// P target in world coordinates
	Eigen::VectorXd P_target;
	// Epsilon
	double Epsilon = 0.01;
	// The difference between P-target and P_endEffector
	Eigen::VectorXd Error;
	Eigen::VectorXd start;
	Eigen::VectorXd end;
	int lerp_iteration = 0;
	int spline_lerp = 0;



};