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
	return 0;
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