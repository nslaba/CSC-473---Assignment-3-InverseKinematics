#pragma once

#include "Jacobian.h"
#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include "Particle.h"
#include "shared/opengl.h"
#include <vector>
#include <glm/glm.hpp>
#include "rAngles.h"


class BetaSolver
{
public:
	BetaSolver(Jacobian j, glm::vec3 dtX);

	Eigen::MatrixXd beta;

	// Solve for beta
	Eigen::Vector3d betaSolver(Jacobian j, glm::vec3 dtX);

protected:
	
	
	Eigen::Vector3d Y; // temp Y vector to help solve for Beta
	Eigen::Vector3d Beta; // final solution
	Eigen::Matrix3Xd Lower; // lower triangular of JJt
	Eigen::Matrix3Xd Upper; // Upper triangular of JJt

};

