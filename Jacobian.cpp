#include "Jacobian.h"

Jacobian::Jacobian(rAngles angles, double L1, double L2, double L3, Eigen::Vector4d& point): jacobian(3, 7)
{
	setColumn1(angles, L1, L2, L3, point);
	setColumn2(angles, L1, L2, L3, point);
	setColumn3(angles, L1, L2, L3, point);
	setColumn4(angles, L1, L2, L3, point);
	setColumn5(angles, L1, L2, L3, point);
	setColumn6(angles, L1, L2, L3, point);
	setColumn7(angles, L1, L2, L3, point);
	
}



void Jacobian::setColumn1(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point)
{
	
	// WRIST TRANSFORM
	xRoll(angles.theta6, point);
	yRoll(angles.theta7, point);
	tWrist(trWrist, point);
	
	
	// ELBOW TRANSFORM
	xRoll(angles.theta4, point);
	yRoll(angles.theta5, point);
	tElbow(trElbow, point);
	
	//SHOULDER TRANSFORM
	dtxRoll(angles.theta1, point);
	dtyRoll(angles.theta2, point);
	zRoll(angles.theta3, point);
	tShoulder(trShoulder, point);
	
	jacobian.col(0) << point[0], point[1], point[2];
}
void Jacobian::setColumn2(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point)
{
	
	// WRIST TRANSFORM
	xRoll(angles.theta6, point);
	yRoll(angles.theta7, point);
	tWrist(trWrist, point);
	
	// ELBOW TRANSFORM
	dtxRoll(angles.theta4, point);
	yRoll(angles.theta5, point);
	tElbow(trElbow, point);
	
	//SHOULDER TRANSFORM
	xRoll(angles.theta1, point);
	dtyRoll(angles.theta2, point);
	zRoll(angles.theta3, point);
	tShoulder(trShoulder, point);
	
	//animTcl::OutputMessage("Column 2 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	jacobian.col(1) << point[0], point[1], point[2];
}
void Jacobian::setColumn3(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point)
{
	
	// WRIST TRANSFORM
	xRoll(angles.theta6, point);
	yRoll(angles.theta7, point);
	tWrist(trWrist, point);
	
	// ELBOW TRANSFORM
	dtxRoll(angles.theta4, point);
	dtyRoll(angles.theta5, point);
	tElbow(trElbow, point);
	
	//SHOULDER TRANSFORM
	xRoll(angles.theta1, point);
	yRoll(angles.theta2, point);
	dtzRoll(angles.theta3, point);
	tShoulder(trShoulder, point);
	
	//animTcl::OutputMessage("Column 3 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	jacobian.col(2) << point[0], point[1], point[2];
}
void Jacobian::setColumn4(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point)
{
	
	// WRIST TRANSFORM
	xRoll(angles.theta6, point);
	yRoll(angles.theta7, point);
	tWrist(trWrist, point);
	
	// ELBOW TRANSFORM
	dtxRoll(angles.theta4, point);
	dtyRoll(angles.theta5, point);
	tElbow(trElbow, point);
	
	//SHOULDER TRANSFORM
	xRoll(angles.theta1, point);
	yRoll(angles.theta2, point);
	zRoll(angles.theta3, point);
	tShoulder(trShoulder, point);
	
	//animTcl::OutputMessage("Column 4 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	jacobian.col(3) << point[0], point[1], point[2];
}
void Jacobian::setColumn5(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point)
{
	
	// WRIST TRANSFORM
	xRoll(angles.theta6, point);
	yRoll(angles.theta7, point);
	tWrist(trWrist, point);
	
	// ELBOW TRANSFORM
	xRoll(angles.theta4, point);
	dtyRoll(angles.theta5, point);
	tElbow(trElbow, point);
	
	//SHOULDER TRANSFORM
	xRoll(angles.theta1, point);
	yRoll(angles.theta2, point);
	zRoll(angles.theta3, point);
	tShoulder(trShoulder, point);
	
	//animTcl::OutputMessage("Column 5 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	jacobian.col(4) << point[0], point[1], point[2];
}
void Jacobian::setColumn6(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point)
{
	
	// WRIST TRANSFORM
	dtxRoll(angles.theta6, point);
	yRoll(angles.theta7, point);
	tWrist(trWrist, point);
	
	// ELBOW TRANSFORM
	xRoll(angles.theta4, point);
	yRoll(angles.theta5, point);
	tElbow(trElbow, point);
	
	//SHOULDER TRANSFORM
	xRoll(angles.theta1, point);
	yRoll(angles.theta2, point);
	zRoll(angles.theta3, point);
	tShoulder(trShoulder, point);
	
	//animTcl::OutputMessage("Column 6 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	jacobian.col(5) << point[0], point[1], point[2];
}
void Jacobian::setColumn7(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d& point)
{
	
	// WRIST TRANSFORM
	xRoll(angles.theta6, point);
	dtyRoll(angles.theta7, point);
	tWrist(trWrist, point);
	// ELBOW TRANSFORM
	xRoll(angles.theta4, point);
	yRoll(angles.theta5, point);
	tElbow(trElbow, point);
	//SHOULDER TRANSFORM
	xRoll(angles.theta1, point);
	yRoll(angles.theta2, point);
	zRoll(angles.theta3, point);
	tShoulder(trShoulder, point);
	//animTcl::OutputMessage("Column 7 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	// Set the Jacobian column
	jacobian.col(6) << point[0], point[1], point[2];
}

void Jacobian::tShoulder(double translation, Eigen::Vector4d& point)
{
	Eigen::Matrix4d tShoulder;
	tShoulder<<
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	;
	
	point = tShoulder * point;
}

void Jacobian::tElbow(double translation, Eigen::Vector4d& point)
{
	Eigen::Matrix4d tElbow;
	tElbow <<
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	;
	
	point = tElbow * point;
}

void Jacobian::tWrist(double translation, Eigen::Vector4d& point)
{
	Eigen::Matrix4d tWrist;
	tWrist <<
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	;
	
	point = tWrist * point;
}

void Jacobian::xRoll(double theta, Eigen::Vector4d &point)
{
	// THETA IN RADIANS
	theta *= (float)M_PI / 180.0;

	Eigen::Matrix4d xRoll;
	xRoll <<
		1.0, 0.0, 0.0, 0.0,
		0.0, cos(theta), -sin(theta), 0,
		0.0, sin(theta), cos(theta), 0.0,
		0.0, 0.0, 0.0, 1.0
	;
	
	point = xRoll * point;
}
void Jacobian::yRoll(double theta, Eigen::Vector4d& point)
{

	// THETA IS IN RADIANS
	theta *= (float)M_PI / 180.0;

	Eigen::Matrix4d yRoll;
	yRoll <<
		cos(theta), 0.0, sin(theta), 0.0,
		0.0, 1.0, 0.0, 0.0,
		-sin(theta), 0.0, cos(theta), 0.0,
		0.0, 0.0, 0.0, 1.0
	;

	point = yRoll * point;
}
void Jacobian::zRoll(double theta, Eigen::Vector4d& point)
{

	// THETA IS IN RADIANS
	theta *= (float)M_PI / 180.0;

	Eigen::Matrix4d zRoll;
	zRoll <<
		cos(theta), -sin(theta), 0.0, 0.0,
		sin(theta), cos(theta), 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	;

	point = zRoll * point;
}
void Jacobian::dtxRoll(double theta, Eigen::Vector4d &point)
{
	// THETA IS IN RADIANS
	theta *= (float)M_PI / 180.0;
	Eigen::Matrix4d dtxRoll;
	
	dtxRoll <<
		0.0, 0.0, 0.0, 0.0,
		0.0, -sin(theta), -cos(theta), 0.0,
		0.0, cos(theta), -sin(theta), 0.0,
		0.0, 0.0, 0.0, 0.0
	;

	point = dtxRoll * point;
}
void Jacobian::dtyRoll(double theta, Eigen::Vector4d& point)
{
	// THETA IS IN RADIANS
	theta *= (float)M_PI / 180.0;

	Eigen::Matrix4d dtyRoll;
	dtyRoll <<
		-sin(theta), 0.0, cos(theta), 0.0,
		0.0, 0.0, 0.0, 0.0,
		-cos(theta), 0.0, -sin(theta), 0.0,
		0.0, 0.0, 0.0, 0.0
	;

	point = dtyRoll * point;
}
void Jacobian::dtzRoll(double theta, Eigen::Vector4d& point)
{
	// THETA IS IN RADIANS
	theta *= (float)M_PI / 180.0;

	Eigen::Matrix4d dtzRoll;
	dtzRoll <<
		-sin(theta), -cos(theta), 0.0, 0.0,
		cos(theta), -sin(theta), 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0
		;
	point = dtzRoll * point;
}
