#include "Jacobian.h"

Jacobian::Jacobian() 
{

}

glm::dvec4 Jacobian::setColumn1(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point)
{
	glm::dvec4 pointWorld;
	// WRIST TRANSFORM
	pointWorld = tWrist(trWrist, (yRoll(angles.theta7, (xRoll(angles.theta6, point)))));
	// ELBOW TRANSFORM
	pointWorld = tElbow(trElbow, (yRoll(angles.theta5, ( xRoll(angles.theta4, point)))));
	//SHOULDER TRANSFORM
	pointWorld = tShoulder(trShoulder, (zRoll(angles.theta3, (yRoll(angles.theta2, (dtxRoll(angles.theta1, point)))))));

	return pointWorld;
}
glm::dvec4 Jacobian::setColumn2(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point)
{
	glm::dvec4 pointWorld;
	// WRIST TRANSFORM
	pointWorld = tWrist(trWrist, (yRoll(angles.theta7, (xRoll(angles.theta6, point)))));
	// ELBOW TRANSFORM
	pointWorld = tElbow(trElbow, (yRoll(angles.theta5, (xRoll(angles.theta4, point)))));
	//SHOULDER TRANSFORM
	pointWorld = tShoulder(trShoulder, (zRoll(angles.theta3, (dtyRoll(angles.theta2, (xRoll(angles.theta1, point)))))));

	return pointWorld;
}
glm::dvec4 Jacobian::setColumn3(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point)
{
	glm::dvec4 pointWorld;
	// WRIST TRANSFORM
	pointWorld = tWrist(trWrist, (yRoll(angles.theta7, (xRoll(angles.theta6, point)))));
	// ELBOW TRANSFORM
	pointWorld = tElbow(trElbow, (yRoll(angles.theta5, (xRoll(angles.theta4, point)))));
	//SHOULDER TRANSFORM
	pointWorld = tShoulder(trShoulder, (dtzRoll(angles.theta3, (yRoll(angles.theta2, (xRoll(angles.theta1, point)))))));

	return pointWorld;
}
glm::dvec4 Jacobian::setColumn4(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point)
{
	glm::dvec4 pointWorld;
	// WRIST TRANSFORM
	pointWorld = tWrist(trWrist, (yRoll(angles.theta7, (xRoll(angles.theta6, point)))));
	// ELBOW TRANSFORM
	pointWorld = tElbow(trElbow, (yRoll(angles.theta5, (dtxRoll(angles.theta4, point)))));
	//SHOULDER TRANSFORM
	pointWorld = tShoulder(trShoulder, (zRoll(angles.theta3, (yRoll(angles.theta2, (xRoll(angles.theta1, point)))))));

	return pointWorld;
}
glm::dvec4 Jacobian::setColumn5(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point)
{
	glm::dvec4 pointWorld;
	// WRIST TRANSFORM
	pointWorld = tWrist(trWrist, (yRoll(angles.theta7, (xRoll(angles.theta6, point)))));
	// ELBOW TRANSFORM
	pointWorld = tElbow(trElbow, (dtyRoll(angles.theta5, (xRoll(angles.theta4, point)))));
	//SHOULDER TRANSFORM
	pointWorld = tShoulder(trShoulder, (zRoll(angles.theta3, (yRoll(angles.theta2, (xRoll(angles.theta1, point)))))));

	return pointWorld;
}
glm::dvec4 Jacobian::setColumn6(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point)
{
	glm::dvec4 pointWorld;
	// WRIST TRANSFORM
	pointWorld = tWrist(trWrist, (yRoll(angles.theta7, (dtxRoll(angles.theta6, point)))));
	// ELBOW TRANSFORM
	pointWorld = tElbow(trElbow, (yRoll(angles.theta5, (xRoll(angles.theta4, point)))));
	//SHOULDER TRANSFORM
	pointWorld = tShoulder(trShoulder, (zRoll(angles.theta3, (yRoll(angles.theta2, (xRoll(angles.theta1, point)))))));

	return pointWorld;
}
glm::dvec4 Jacobian::setColumn7(rAngles angles, double trShoulder, double trElbow, double trWrist, glm::dvec4 point)
{
	glm::dvec4 pointWorld;
	// WRIST TRANSFORM
	pointWorld = tWrist(trWrist, (dtyRoll(angles.theta7, (xRoll(angles.theta6, point)))));
	// ELBOW TRANSFORM
	pointWorld = tElbow(trElbow, (yRoll(angles.theta5, (xRoll(angles.theta4, point)))));
	//SHOULDER TRANSFORM
	pointWorld = tShoulder(trShoulder, (zRoll(angles.theta3, (yRoll(angles.theta2, (xRoll(angles.theta1, point)))))));

	return pointWorld;
}

glm::vec4 Jacobian::tShoulder(double translation, glm::dvec4 point)
{
	glm::dmat4x4 tShoulder{
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	return tShoulder * point;
}

glm::vec4 Jacobian::tElbow(double translation, glm::dvec4 point)
{
	glm::dmat4x4 tShoulder{
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	return tShoulder * point;
}

glm::vec4 Jacobian::tWrist(double translation, glm::dvec4 point)
{
	glm::dmat4x4 tShoulder{
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, -translation,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	return tShoulder * point;
}

glm::dvec4 Jacobian::xRoll(double theta, glm::vec4 point)
{
	// THETA IS IN RADIANS
	glm::dmat4x4 xRoll{
		1.0, 0.0, 0.0, 0.0,
		0.0, cos(theta), -sin(theta), 0,
		0.0, sin(theta), cos(theta), 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	return xRoll * point;
}
glm::dvec4 Jacobian::yRoll(double theta, glm::vec4 point)
{

	// THETA IS IN RADIANS
	glm::dmat4x4 yRoll{
		cos(theta), 0.0, sin(theta), 1.0,
		0.0, 1.0, 0.0, 0.0,
		-sin(theta), 0.0, cos(theta), 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	return yRoll * point;
}
glm::dvec4 Jacobian::zRoll(double theta, glm::vec4 point)
{

	// THETA IS IN RADIANS
	glm::dmat4x4 zRoll{
		cos(theta), -sin(theta), 0.0, 0.0,
		sin(theta), cos(theta), 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	return zRoll * point;
}
glm::dvec4 Jacobian::dtxRoll(double theta, glm::vec4 point)
{
	// THETA IS IN RADIANS
	glm::dmat4x4 dtxRoll{
		0.0, 0.0, 0.0, 0.0,
		0.0, -sin(theta), -cos(theta), 0.0,
		0.0, cos(theta), -sin(theta), 0.0,
		0.0, 0.0, 0.0, 0.0
	};

	return dtxRoll * point;
}
glm::dvec4 Jacobian::dtyRoll(double theta, glm::vec4 point)
{
	// THETA IS IN RADIANS
	glm::dmat4x4 dtyRoll{
		-sin(theta), 0.0, cos(theta), 0.0,
		0.0, 0.0, 0.0, 0.0,
		-cos(theta), 0.0, -sin(theta), 0.0,
		0.0, 0.0, 0.0, 0.0
	};

	return dtyRoll * point;
}
glm::dvec4 Jacobian::dtzRoll(double theta, glm::vec4 point)
{
	// THETA IS IN RADIANS
	glm::dmat4x4 dtzRoll{
		-sin(theta), -cos(theta), 0.0, 0.0,
		cos(theta), -sin(theta), 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0
	};

	return dtzRoll * point;
}