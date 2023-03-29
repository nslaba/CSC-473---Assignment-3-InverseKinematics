#include "Jacobian.h"
void printMatrixJac(Eigen::MatrixXd mat)
{
	animTcl::OutputMessage("\nJacobian by row: ");
	for (int i = 0; i < 3; i++) {
		animTcl::OutputMessage("%f %f %f %f %f %f %f\n", mat.row(i)[0], mat.row(i)[1], mat.row(i)[2], mat.row(i)[3], mat.row(i)[4], mat.row(i)[5], mat.row(i)[6]);
	}
	animTcl::OutputMessage("\n\n");
	animTcl::OutputMessage("\nJacobian by column: ");
	for (int i = 0; i < 7; i++) {
		animTcl::OutputMessage("%f %f %f\n", mat.col(i)[0], mat.col(i)[1], mat.col(i)[2]);
	}
	animTcl::OutputMessage("\n\n");
}
Jacobian::Jacobian(rAngles angles, double L1, double L2, double L3, Eigen::Vector4d& point): jacobian(3, 7)
{
	setColumn1(angles, L1, L2, L3, point);
	setColumn2(angles, L1, L2, L3, point);
	setColumn3(angles, L1, L2, L3, point);
	setColumn4(angles, L1, L2, L3, point);
	setColumn5(angles, L1, L2, L3, point);
	setColumn6(angles, L1, L2, L3, point);
	setColumn7(angles, L1, L2, L3, point);
	printMatrixJac(jacobian);
}



void Jacobian::setColumn1(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point)
{
	point = translateZ(-3) * tShoulder(trShoulder) * zRoll(angles.theta3) * yRoll(angles.theta2) * dtxRoll(angles.theta1)
		* tElbow(trElbow) * yRoll(angles.theta5) * xRoll(angles.theta4)
		* tWrist(trWrist) * yRoll(angles.theta7) * xRoll(angles.theta6) * point;
	/*point = translateZ(-3) * tShoulder(trShoulder) * zRoll(angles.theta3) * yRoll(angles.theta2) * dtxRoll(angles.theta1)
		* tElbow(trElbow) * yRoll(angles.theta5) * xRoll(angles.theta4)
		* tWrist(trWrist) * yRoll(angles.theta7) * xRoll(angles.theta6) * point;*/

	jacobian.col(0) << point[0], point[1], point[2];
}
void Jacobian::setColumn2(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point)
{
	//point = yRoll(angles.theta7) * xRoll(angles.theta6) * point;
	point = translateZ(-3) * tShoulder(trShoulder) * zRoll(angles.theta3) * dtyRoll(angles.theta2) * xRoll(angles.theta1)
		* tElbow(trElbow) * yRoll(angles.theta5) * xRoll(angles.theta4)
		* tWrist(trWrist) * yRoll(angles.theta7) * xRoll(angles.theta6) * point;
	//animTcl::OutputMessage("Column 2 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	jacobian.col(1) << point[0], point[1], point[2];
}
void Jacobian::setColumn3(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point)
{
	
	//point = yRoll(angles.theta7) * xRoll(angles.theta6) * point;
	point = translateZ(-3) * tShoulder(trShoulder) * dtzRoll(angles.theta3) * yRoll(angles.theta2) * xRoll(angles.theta1)
		* tElbow(trElbow) * yRoll(angles.theta5) * xRoll(angles.theta4)
		* tWrist(trWrist) * yRoll(angles.theta7) * xRoll(angles.theta6) * point;
	//animTcl::OutputMessage("Column 3 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	jacobian.col(2) << point[0], point[1], point[2];
}
void Jacobian::setColumn4(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point)
{
	//point = yRoll(angles.theta7) * xRoll(angles.theta6) * point;
	point = translateZ(-3) * tShoulder(trShoulder) * zRoll(angles.theta3) * yRoll(angles.theta2) * xRoll(angles.theta1)
		* tElbow(trElbow) * yRoll(angles.theta5) * dtxRoll(angles.theta4)
		* tWrist(trWrist) * yRoll(angles.theta7) * xRoll(angles.theta6) * point;
	//animTcl::OutputMessage("Column 4 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	jacobian.col(3) << point[0], point[1], point[2];
}
void Jacobian::setColumn5(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point)
{
	
	//point = yRoll(angles.theta7) * xRoll(angles.theta6) * point;
	point = translateZ(-3) * tShoulder(trShoulder) * zRoll(angles.theta3) * yRoll(angles.theta2) * xRoll(angles.theta1)
		* tElbow(trElbow) * dtyRoll(angles.theta5) * xRoll(angles.theta4)
		* tWrist(trWrist) * yRoll(angles.theta7) * xRoll(angles.theta6) * point;
	//animTcl::OutputMessage("Column 5 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	jacobian.col(4) << point[0], point[1], point[2];
}
void Jacobian::setColumn6(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point)
{

	
	//point = yRoll(angles.theta7) * dtxRoll(angles.theta6) * point;
	point = translateZ(-3) * tShoulder(trShoulder) * zRoll(angles.theta3) * yRoll(angles.theta2) * xRoll(angles.theta1)
		* tElbow(trElbow) * yRoll(angles.theta5) * xRoll(angles.theta4)
		* tWrist(trWrist) * yRoll(angles.theta7) * dtxRoll(angles.theta6) * point;
	//animTcl::OutputMessage("Column 6 values in jacobian are %f, %f, %f", pointWorld[0], pointWorld[1], pointWorld[2]);
	jacobian.col(5) << point[0], point[1], point[2];
}
void Jacobian::setColumn7(rAngles angles, double trShoulder, double trElbow, double trWrist, Eigen::Vector4d point)
{
	//point = xRoll(angles.theta6) * point;

	point = translateZ(-3) * tShoulder(trShoulder) * zRoll(angles.theta3) * yRoll(angles.theta2) * xRoll(angles.theta1)
		* tElbow(trElbow) * yRoll(angles.theta5) * xRoll(angles.theta4)
		* tWrist(trWrist) * dtyRoll(angles.theta7) * xRoll(angles.theta6) * point;
	// Set the Jacobian column
	jacobian.col(6) << point[0], point[1], point[2];
}

Eigen::Matrix4d Jacobian::translateZ(double translation)
{
	Eigen::Matrix4d translateZ;
	translateZ <<
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, -translation,
		0.0, 0.0, 0.0, 1.0
		;

	return translateZ;
}

Eigen::Matrix4d Jacobian::tShoulder(double translation)
{
	Eigen::Matrix4d tShoulder;
	tShoulder<<
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	;
	
	return tShoulder;
}

Eigen::Matrix4d Jacobian::tElbow(double translation)
{
	Eigen::Matrix4d tElbow;
	tElbow <<
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	;
	
	return tElbow;
}

Eigen::Matrix4d Jacobian::tWrist(double translation)
{
	Eigen::Matrix4d tWrist;
	tWrist <<
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	;
	
	return tWrist;
}

Eigen::Matrix4d Jacobian::xRoll(double theta)
{
	// THETA IN RADIANS
	theta *= 3.141592653589 / 180.0;

	Eigen::Matrix4d xRoll;
	xRoll <<
		1.0, 0.0, 0.0, 0.0,
		0.0, cos(theta), -sin(theta), 0,
		0.0, sin(theta), cos(theta), 0.0,
		0.0, 0.0, 0.0, 1.0
	;
	
	return xRoll;
}
Eigen::Matrix4d Jacobian::yRoll(double theta)
{

	// THETA IS IN RADIANS
	theta *= 3.141592653589 / 180.0;

	Eigen::Matrix4d yRoll;
	yRoll <<
		cos(theta), 0.0, sin(theta), 0.0,
		0.0, 1.0, 0.0, 0.0,
		-sin(theta), 0.0, cos(theta), 0.0,
		0.0, 0.0, 0.0, 1.0
	;

	return yRoll;
}
Eigen::Matrix4d Jacobian::zRoll(double theta)
{

	// THETA IS IN RADIANS
	theta *= 3.141592653589 / 180.0;

	Eigen::Matrix4d zRoll;
	zRoll <<
		cos(theta), -sin(theta), 0.0, 0.0,
		sin(theta), cos(theta), 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	;

	return zRoll;
}
Eigen::Matrix4d Jacobian::dtxRoll(double theta)
{
	// THETA IS IN RADIANS
	theta *= 3.141592653589 / 180.0;
	Eigen::Matrix4d dtxRoll;
	
	dtxRoll <<
		0.0, 0.0, 0.0, 0.0,
		0.0, -sin(theta), -cos(theta), 0.0,
		0.0, cos(theta), -sin(theta), 0.0,
		0.0, 0.0, 0.0, 0.0
	;

	return dtxRoll;
}
Eigen::Matrix4d Jacobian::dtyRoll(double theta)
{
	// THETA IS IN RADIANS
	theta *= 3.141592653589 / 180.0;

	Eigen::Matrix4d dtyRoll;
	dtyRoll <<
		-sin(theta), 0.0, cos(theta), 0.0,
		0.0, 0.0, 0.0, 0.0,
		-cos(theta), 0.0, -sin(theta), 0.0,
		0.0, 0.0, 0.0, 0.0
	;

	return dtyRoll;
}
Eigen::Matrix4d Jacobian::dtzRoll(double theta)
{
	// THETA IS IN RADIANS
	theta *= 3.141592653589 / 180.0;

	Eigen::Matrix4d dtzRoll;
	dtzRoll <<
		-sin(theta), -cos(theta), 0.0, 0.0,
		cos(theta), -sin(theta), 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0
		;
	return dtzRoll;
}
