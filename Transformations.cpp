#include "Transformations.h"

void printMatrix(Eigen::MatrixXd mat)
{

	for (int i = 0; i < 4; i++) {
		animTcl::OutputMessage("%f %f %f %f\n", mat.row(i)[0], mat.row(i)[1], mat.row(i)[2], mat.row(i)[3]);
	}
	animTcl::OutputMessage("\n\n");
}
Transformations::Transformations(rAngles angles, double L1, double L2, double L3): matrixTransform(4,4)
{
	// TEMPORARY TRANSLATION TO MATCH WITH NIMA
	//matrixTransform = translateX(-0.6);
	//matrixTransform = matrixTransform * translateY(-1.5);
	///////////////////////////
	/////DONT FORGET TO FIX!!!!!
	matrixTransform =  translateZ(-3);
	matrixTransform = matrixTransform * xRoll(angles.theta1);
	
	animTcl::OutputMessage("after sholder xRoll:");
	printMatrix(matrixTransform);
	matrixTransform = matrixTransform * yRoll(angles.theta2);
	
	animTcl::OutputMessage("after shoulder xRoll and yroll:");
	printMatrix(matrixTransform);
	matrixTransform = matrixTransform * zRoll(angles.theta3);
	
	animTcl::OutputMessage("\nafter shoulder xRoll yroll and zroll:");
	printMatrix(matrixTransform);
	matrixTransform = matrixTransform * translateY(L1);
	
	animTcl::OutputMessage("\nat elbow:");
	printMatrix(matrixTransform);
	matrixTransform = matrixTransform * xRoll(angles.theta4);
	matrixTransform = matrixTransform * yRoll(angles.theta5);
	
	animTcl::OutputMessage("\nafter elbow xRoll and yroll:");
	printMatrix(matrixTransform);
	matrixTransform = matrixTransform * translateY(L2);
	
	animTcl::OutputMessage("\nat wrist");
	printMatrix(matrixTransform);
	matrixTransform = matrixTransform * zRoll(angles.theta6);
	matrixTransform = matrixTransform * yRoll(angles.theta7);
	
	animTcl::OutputMessage("\nafter wrist xRoll and yRoll:");
	printMatrix(matrixTransform);
	matrixTransform = matrixTransform *translateY(L3);
	
	animTcl::OutputMessage("\nfinal transform matrix at wrist");
	printMatrix(matrixTransform);
}

Eigen::Matrix4d Transformations::translateZ(double translation)
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

Eigen::Matrix4d Transformations::translateY(double translation)
{
	Eigen::Matrix4d translateY;
	translateY <<
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
		;

	return translateY;
}

Eigen::Matrix4d Transformations::translateX(double translation)
{
	Eigen::Matrix4d translateX;
	translateX <<
		1.0, 0.0, 0.0, -translation,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
		;

	return translateX;
}

Eigen::Matrix4d Transformations::xRoll(double theta)
{
	
	Eigen::Matrix4d xRoll;
	xRoll <<
		1.0, 0.0, 0.0, 0.0,
		0.0, cos(theta), -sin(theta), 0.0,
		0.0, sin(theta), cos(theta), 0.0,
		0.0, 0.0, 0.0, 1.0
		;
	
	return xRoll;
}
Eigen::Matrix4d Transformations::yRoll(double theta)
{

	
	Eigen::Matrix4d yRoll;
	yRoll <<
		cos(theta), 0.0, sin(theta), 0.0,
		0.0, 1.0, 0.0, 0.0,
		-sin(theta), 0.0, cos(theta), 0.0,
		0.0, 0.0, 0.0, 1.0
		;
	
	return yRoll;
}
Eigen::Matrix4d Transformations::zRoll(double theta)
{

	
	Eigen::Matrix4d zRoll;
	zRoll <<
		cos(theta), -sin(theta), 0.0, 0.0,
		sin(theta), cos(theta), 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
		;
	
	return zRoll;
}
Eigen::Matrix4d Transformations::dtxRoll(double theta)
{
	
	Eigen::Matrix4d dtxRoll;
	dtxRoll <<
		0.0, 0.0, 0.0, 0.0,
		0.0, -sin(theta), -cos(theta), 0.0,
		0.0, cos(theta), -sin(theta), 0.0,
		0.0, 0.0, 0.0, 0.0
		;

	return dtxRoll;
}
Eigen::Matrix4d Transformations::dtyRoll(double theta)
{
	
	Eigen::Matrix4d dtyRoll;
	dtyRoll <<
		-sin(theta), 0.0, cos(theta), 0.0,
		0.0, 0.0, 0.0, 0.0,
		-cos(theta), 0.0, -sin(theta), 0.0,
		0.0, 0.0, 0.0, 0.0
		;

	return dtyRoll;
}
Eigen::Matrix4d Transformations::dtzRoll(double theta)
{
	
	Eigen::Matrix4d dtzRoll;
	dtzRoll <<
		-sin(theta), -cos(theta), 0.0, 0.0,
		cos(theta), -sin(theta), 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0
		;
	return dtzRoll;
}

