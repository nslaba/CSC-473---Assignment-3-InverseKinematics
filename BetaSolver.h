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

	Eigen::MatrixXd betaSolver;
	// getters and setters


protected:
	
	// Translations
	
	// Rotations
	
};

