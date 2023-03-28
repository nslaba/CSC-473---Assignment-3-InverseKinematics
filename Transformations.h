#pragma once

#include <eigen-3.4.0/Eigen/Dense>
#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include "shared/opengl.h"
#include <glm/glm.hpp>
#include "rAngles.h"

class Transformations
{
public:
	// default constructor
	Transformations() = default;
	
	// Translations
	static void translateY(double translation, Eigen::VectorXd& point);
	static void translateX(double translation, Eigen::VectorXd& point);

	// Rotations
	static void xRoll(double theta, Eigen::VectorXd& point);
	static void yRoll(double theta, Eigen::VectorXd& point);
	static void zRoll(double theta, Eigen::VectorXd& point);
	static void dtxRoll(double theta, Eigen::VectorXd& point);
	static void dtyRoll(double theta, Eigen::VectorXd& point);
	static void dtzRoll(double theta, Eigen::VectorXd& point);

	// Create a series of matrices to multiply together to get one correct transformation matrix

protected:

};