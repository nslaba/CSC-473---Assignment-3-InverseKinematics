#pragma once

//#include "Transformations.h"
#include <eigen-3.4.0/Eigen/Dense>
#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <GLmodel/GLmodel.h>
#include "shared/opengl.h"
#include <vector>
#include <glm/glm.hpp>
#include "rAngles.h"


class Jacobian 
{
public:
	// default constructor
	Jacobian() = default;
	Jacobian(rAngles angles, double L1, double L2, double L3, Eigen::Vector4d& point);

	Eigen::MatrixXd jacobian;
	// getters and setters
	
	void setColumn1(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point);
	void setColumn2(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point);
	void setColumn3(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point);
	void setColumn4(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point);
	void setColumn5(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point);
	void setColumn6(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point);
	void setColumn7(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point);


	// Translations
	void tShoulder(double translation, Eigen::Vector4d& point);
	void tElbow(double translation, Eigen::Vector4d& point);
	void tWrist(double translation, Eigen::Vector4d& point);

	// Rotations
	void xRoll(double theta, Eigen::Vector4d& point);
	void yRoll(double theta, Eigen::Vector4d& point);
	void zRoll(double theta, Eigen::Vector4d& point);
	void dtxRoll(double theta, Eigen::Vector4d& point);
	void dtyRoll(double theta, Eigen::Vector4d& point);
	void dtzRoll(double theta, Eigen::Vector4d& point);
protected:
	

	
};

