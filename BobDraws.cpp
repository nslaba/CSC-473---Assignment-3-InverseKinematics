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
	// At time == 0, make hand move to the start of the spline
	if (!time)
	{


	}

	//if (!drawingPath->controlPoints[cPointID +1].empty)

		Ptarget = drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t);
		dtX = Ptarget.point - P.point;
		
		// PLUG CURR ANGLES INTO JACOBIAN
		Jacobian j = Jacobian(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, glm::vec4{ P.point,1 });
		
		// Find BETA
		BetaSolver beta = BetaSolver(j, dtX);
		// FIND PSEUDO INVERSE OF JACOBIAN BASED ON BETA
		Eigen::VectorXd dtThetas = j.jacobian.transpose() * beta.beta;
		
		// turn into degrees
		dtThetas *= 180.0 / M_PI;
		// UPDATE THETAS
		m_bob->angles.theta1 = m_bob->angles.theta1 + dtThetas[0];
		m_bob->angles.theta2 = m_bob->angles.theta2 + dtThetas[1];
		m_bob->angles.theta3 = m_bob->angles.theta3 + dtThetas[2];
		m_bob->angles.theta4 = m_bob->angles.theta4 + dtThetas[3];
		m_bob->angles.theta5 = m_bob->angles.theta5 + dtThetas[4];
		m_bob->angles.theta6 = m_bob->angles.theta6 + dtThetas[5];
		m_bob->angles.theta7 = m_bob->angles.theta7 + dtThetas[6];
		
		// UPDATE P 
		P = Ptarget;	
//	}
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
	ControlPoint initial = ControlPoint();
	initial.empty = false;
	initial.point = glm::vec3(0.0, -(m_bob->L1 + m_bob->L2 + m_bob->L3) + m_bob->torsoHeight / 4.0, m_bob->z); //initial position of end effector
	P = initial;
	Ptarget = drawingPath->controlPoints[0];
	//P = drawingPath->controlPoints[0];
	//Ptarget = drawingPath->getNext(drawingPath->controlPoints[0], drawingPath->controlPoints[1], t);
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