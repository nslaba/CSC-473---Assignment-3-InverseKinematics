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
	static void translateY(double translation, Eigen::Vector4d& point);


	// Rotations
	static void xRoll(double theta, Eigen::Vector4d& point);
	static void yRoll(double theta, Eigen::Vector4d& point);
	static void zRoll(double theta, Eigen::Vector4d& point);
	static void dtxRoll(double theta, Eigen::Vector4d& point);
	static void dtyRoll(double theta, Eigen::Vector4d& point);
	static void dtzRoll(double theta, Eigen::Vector4d& point);

	// Create a series of matrices to multiply together to get one correct transformation matrix

protected:

};