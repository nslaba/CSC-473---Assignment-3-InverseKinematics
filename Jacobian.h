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
	
	void setColumn1(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point);
	void setColumn2(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point);
	void setColumn3(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point);
	void setColumn4(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point);
	void setColumn5(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point);
	void setColumn6(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point);
	void setColumn7(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point);


	// Translations
	Eigen::Matrix4d translateZ(double translation);
	Eigen::Matrix4d tShoulder(double translation);
	Eigen::Matrix4d tElbow(double translation);
	Eigen::Matrix4d tWrist(double translation);

	// Rotations
	Eigen::Matrix4d xRoll(double theta);
	Eigen::Matrix4d yRoll(double theta);
	Eigen::Matrix4d zRoll(double theta);
	Eigen::Matrix4d dtxRoll(double theta);
	Eigen::Matrix4d dtyRoll(double theta);
	Eigen::Matrix4d dtzRoll(double theta);
protected:
	

	
};

