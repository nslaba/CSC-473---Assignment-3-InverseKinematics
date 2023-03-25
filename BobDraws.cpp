#include "BobDraws.h"

BobDraws::BobDraws(const std::string& name, HermiteSpline* drawing, Bob* bob) :
	BaseSimulator(name),
	drawingPath(drawing),
	m_bob(bob)
{
}	// BobDraws

BobDraws::~BobDraws()
{
}	// BobDraws::~BobDraws

int BobDraws::step(double time)
{
	

	if (!drawingPath->controlPoints[cPointID + 1].empty)
	{

		Ptarget = drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t);
		dtX = Ptarget.point - P.point;

		// PLUG CURR ANGLES INTO JACOBIAN
		Jacobian j = Jacobian(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, glm::vec4{ P.point,1 });

		// Find BETA
		BetaSolver beta = BetaSolver(j, dtX);
		// FIND PSEUDO INVERSE OF JACOBIAN BASED ON BETA
		dtThetas = j.jacobian.transpose() * beta.beta;

		// turn into degrees
		dtThetas *= 180.0 / M_PI;
		animTcl::OutputMessage("theta 1 is: %d\n theta 7 is: %d\n", m_bob->angles.theta1, m_bob->angles.theta7);
		// UPDATE THETAS
		m_bob->angles.theta1 = m_bob->angles.theta1 + dtThetas[0];
		m_bob->angles.theta2 = m_bob->angles.theta2 + dtThetas[1];
		m_bob->angles.theta3 = m_bob->angles.theta3 + dtThetas[2];
		m_bob->angles.theta4 = m_bob->angles.theta4 + dtThetas[3];
		m_bob->angles.theta5 = m_bob->angles.theta5 + dtThetas[4];
		m_bob->angles.theta6 = m_bob->angles.theta6 + dtThetas[5];
		m_bob->angles.theta7 = m_bob->angles.theta7 + dtThetas[6];
		animTcl::OutputMessage("dttheta 1 is: %f\n dttheta 2 is: %f\n dttheta 3 is: %f\n dttheta 4 is: %f\n dttheta 5 is: %f\n dttheta 6 is: %f\n dttheta 7 is: %f\n", dtThetas[0], dtThetas[1], dtThetas[2], dtThetas[3], dtThetas[4], dtThetas[5], dtThetas[6]);

		// UPDATE P 
		P = Ptarget;
	}
	// Update t parameter to walk closer towards the next control point
	t++;
	// update control point id when t==1
	if (t) {
		cPointID++;
		t = 0.001;
	}

	return 0;
}

void BobDraws::initializePs() {
	// Initialize resting position
	m_bob->angles = rAngles{ 0.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0 };
	//
	ControlPoint initial = ControlPoint();
	initial.empty = false;
	initial.point = glm::vec3{ 0.0, 0.0, m_bob->z }; 
	
	// Get the correct resting point transformations of the point
	// 1. rotate by theta3 on z
	//transform = glm::rotate(identity, 20 * (float)M_PI / 180, glm::vec3{ 0,0,1.0 });
	//initial.point = transform * glm::vec4{ initial.point, 1 };
	////2. translate on y
	//transform = glm::translate(translateL1, initial.point);
	//// 3. Rotate by theta4 on x
	//transform = glm::rotate(transform, 20 * (float)M_PI / 180, glm::vec3{ 1.0, 0, 0 });
	//initial.point = transform * glm::vec4{ initial.point, 1 };
	//// 4. translate on y twice
	//transform = glm::translate(translateL2L3, initial.point);

	// 1. rotate by theta3 on z

	// 2. translate on y

	// 3. Rotate by theta4 on x

	// 4. translate on y twice
	animTcl::OutputMessage("initial point is: %f %f %f", initial.point.x, initial.point.y, initial.point.z);

	P = initial;
	animTcl::OutputMessage("the point as vec3 values are %f, %f, %f", P.point.x, P.point.y, P.point.z);
	Ptarget = drawingPath->controlPoints[0];

	// update angles to get to the start of the spline
	dtX = Ptarget.point - P.point;

	// PLUG CURR ANGLES INTO JACOBIAN
	animTcl::OutputMessage("initial values are %f, %f, %f", initial.point.x, initial.point.y, initial.point.z);
	animTcl::OutputMessage("the point as vec4 values are %f, %f, %f", P.point.x, P.point.y, P.point.z);
	Jacobian j = Jacobian(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, glm::vec4{ P.point,1 });
	//animTcl::OutputMessage("jacobian x columns are: %d, %d, %d, %d, %d, %d, %d", j.jacobian(0,0), j.jacobian(0,1), j.jacobian(0,2), j.jacobian(0,3), j.jacobian(0,4), j.jacobian(0,5), j.jacobian(0,6));
	// Find BETA
	BetaSolver beta = BetaSolver(j, dtX);
	// FIND PSEUDO INVERSE OF JACOBIAN BASED ON BETA
	dtThetas = j.jacobian.transpose() * beta.beta;
	animTcl::OutputMessage("Beta x columns are: %f, %f, %f", beta.beta(0), beta.beta(1), beta.beta(2));
	// turn into degrees
	dtThetas *= 180.0 / M_PI;
	animTcl::OutputMessage("theta 1 is: %f\n theta 7 is: %f\n", m_bob->angles.theta1, m_bob->angles.theta7);
	// UPDATE THETAS
	/*m_bob->angles.theta1 = m_bob->angles.theta1 + dtThetas[0];
	m_bob->angles.theta2 = m_bob->angles.theta2 + dtThetas[1];
	m_bob->angles.theta3 = m_bob->angles.theta3 + dtThetas[2];
	m_bob->angles.theta4 = m_bob->angles.theta4 + dtThetas[3];
	m_bob->angles.theta5 = m_bob->angles.theta5 + dtThetas[4];
	m_bob->angles.theta6 = m_bob->angles.theta6 + dtThetas[5];
	m_bob->angles.theta7 = m_bob->angles.theta7 + dtThetas[6];*/
	animTcl::OutputMessage("dttheta 1 is: %f\n dttheta 2 is: %f\n dttheta 3 is: %f\n dttheta 4 is: %f\n dttheta 5 is: %f\n dttheta 6 is: %f\n dttheta 7 is: %f\n", dtThetas[0], dtThetas[1], dtThetas[2], dtThetas[3], dtThetas[4], dtThetas[5], dtThetas[6]);
	
}

int BobDraws::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system: wrong number of params.");
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "read") == 0)
	{ // simulator iksim read spline.txt
		//animTcl::OutputMessage("within read argc is, %d, argv.size is, %d", argc, sizeof(argv));
		if (argc == 2) {
			// call HermiteSpline to load the file argv[3]
			drawingPath->load(argv[1]);
			// Initialize P
			initializePs();
		}
		else {
			// Wrong number of arguments
			animTcl::OutputMessage("Wrong number of arguments");
			return TCL_ERROR;
		}
	}
	glutPostRedisplay();
	return TCL_OK;
}