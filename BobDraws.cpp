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
	// CALCULATE E = Ptarget - Pcurr --> Ptarget and Pcurr are points on spline
	// Also have to account for the condition if at the last control point. not sure how yet
	while (!drawingPath->controlPoints[cPointID +1].empty && glm::length(E) > epsilon)
		//while (!drawingPath->controlPoints[cPointID +1].empty && k < 1)
	{ // WHILE |E| < epsilon do:

		// dtX = K * E --> get dtX based on get next function in 
		Ptarget = drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], 0.01);
		dtX = intermediate - P.point;
		// PLUG CURR ANGLES INTO JACOBIAN
		Jacobian *j = new Jacobian(m_bob->curAngles, m_bob->L1, m_bob->L2, m_bob->L3, glm::vec4{ P.point,1 });
		// FIND PSEUDO INVERSE OF JACOBIAN BASED ON BETA

		// COMPUTE dtTHETA = J+dtX

		// UPDATE THETAS

		// UPDATE P = P+ dtX

		// UPDATE E = Ptarget - P (New error)
	}
	
	t++;
	// update control point id when t==1
	if (t) cPointID++;

	return 0;
}

void BobDraws::initializePs() {
	P = drawingPath->controlPoints[0];
	Ptarget = drawingPath->getNext(drawingPath->controlPoints[0], drawingPath->controlPoints[1], t);
	E = Ptarget.point - P.point;
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