#pragma once

#include <eigen-3.4.0/Eigen/Dense>
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


class Jacobian 
{
public:
	Jacobian(rAngles angles, double L1, double L2, double L3, glm::dvec4 point);

	Eigen::MatrixXd jacobian;
	// getters and setters
	
	glm::dvec4 setColumn1(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point);
	glm::dvec4 setColumn2(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point);
	glm::dvec4 setColumn3(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point);
	glm::dvec4 setColumn4(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point);
	glm::dvec4 setColumn5(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point);
	glm::dvec4 setColumn6(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point);
	glm::dvec4 setColumn7(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point);
protected:
	

	// Translations
	glm::vec4 Jacobian::tShoulder(double translation, glm::dvec4 point);
	glm::vec4 Jacobian::tElbow(double translation, glm::dvec4 point);
	glm::vec4 Jacobian::tWrist(double translation, glm::dvec4 point);

	// Rotations
	glm::dvec4 xRoll(double theta, glm::vec4 point);
	glm::dvec4 yRoll(double theta, glm::vec4 point);
	glm::dvec4 zRoll(double theta, glm::vec4 point);
	glm::dvec4 dtxRoll(double theta, glm::vec4 point);
	glm::dvec4 dtyRoll(double theta, glm::vec4 point);
	glm::dvec4 dtzRoll(double theta, glm::vec4 point);
};

