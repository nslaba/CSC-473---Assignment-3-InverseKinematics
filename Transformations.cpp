#include "Transformations.h"



void Transformations::translateY(double translation, Eigen::Vector4d& point)
{
	Eigen::Matrix4d translateY;
	translateY <<
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
		;

	point = translateY * point;
}



void Transformations::xRoll(double theta, Eigen::Vector4d& point)
{
	// THETA IS IN RADIANS
	Eigen::Matrix4d xRoll;
	xRoll <<
		1.0, 0.0, 0.0, 0.0,
		0.0, cos(theta), -sin(theta), 0,
		0.0, sin(theta), cos(theta), 0.0,
		0.0, 0.0, 0.0, 1.0
		;

	point = xRoll * point;
}
void Transformations::yRoll(double theta, Eigen::Vector4d& point)
{

	// THETA IS IN RADIANS
	Eigen::Matrix4d yRoll;
	yRoll <<
		cos(theta), 0.0, sin(theta), 1.0,
		0.0, 1.0, 0.0, 0.0,
		-sin(theta), 0.0, cos(theta), 0.0,
		0.0, 0.0, 0.0, 1.0
		;

	point = yRoll * point;
}
void Transformations::zRoll(double theta, Eigen::Vector4d& point)
{

	// THETA IS IN RADIANS
	Eigen::Matrix4d zRoll;
	zRoll <<
		cos(theta), -sin(theta), 0.0, 0.0,
		sin(theta), cos(theta), 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
		;

	point = zRoll * point;
}
void Transformations::dtxRoll(double theta, Eigen::Vector4d& point)
{
	// THETA IS IN RADIANS
	Eigen::Matrix4d dtxRoll;
	dtxRoll <<
		0.0, 0.0, 0.0, 0.0,
		0.0, -sin(theta), -cos(theta), 0.0,
		0.0, cos(theta), -sin(theta), 0.0,
		0.0, 0.0, 0.0, 0.0
		;

	point = dtxRoll * point;
}
void Transformations::dtyRoll(double theta, Eigen::Vector4d& point)
{
	// THETA IS IN RADIANS
	Eigen::Matrix4d dtyRoll;
	dtyRoll <<
		-sin(theta), 0.0, cos(theta), 0.0,
		0.0, 0.0, 0.0, 0.0,
		-cos(theta), 0.0, -sin(theta), 0.0,
		0.0, 0.0, 0.0, 0.0
		;

	point = dtyRoll * point;
}
void Transformations::dtzRoll(double theta, Eigen::Vector4d& point)
{
	// THETA IS IN RADIANS
	Eigen::Matrix4d dtzRoll;
	dtzRoll <<
		-sin(theta), -cos(theta), 0.0, 0.0,
		cos(theta), -sin(theta), 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0
		;
	point = dtzRoll * point;
}