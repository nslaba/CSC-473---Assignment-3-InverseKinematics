#include "BobDraws.h"

BobDraws::BobDraws(const std::string& name, HermiteSpline* drawing, Bob* bob) :
	BaseSimulator(name),
	drawingPath(drawing),
	m_bob(bob), endEffector(4), Ptarget(4), dtX_eigen(3), dtThetas(7), intermediateEnd(4)
{											 
}	// BobDraws								  

BobDraws::~BobDraws()						  
{											  
}	// BobDraws::~BobDraws

int BobDraws::step(double time)
{
	/* FIRST OF ALL HANDLE GETTING TO THE SPLINE */
	if (lerping_to_beginning_of_spline < 1.0)
	{
		m_bob->target_point = { intermediateEnd[0], intermediateEnd[1], intermediateEnd[2] };

		animTcl::OutputMessage("intermediateEnd before scaling is as vec3 is: %f, %f, %f", m_bob->temp_end_eff.x, m_bob->temp_end_eff.y, m_bob->temp_end_eff.z);

		// update intermediateEnd to be a scalar of Ptarget.
		intermediateEnd << dtX_eigen[0] * lerping_to_beginning_of_spline, dtX_eigen[1] * lerping_to_beginning_of_spline, dtX_eigen[2] * lerping_to_beginning_of_spline, 0.0;

		//////////////////////////////// TESTING //////////////
		m_bob->target_point = { intermediateEnd[0], intermediateEnd[1], intermediateEnd[2] };
		
		animTcl::OutputMessage("intermediateEnd after scaling as vec3 is: %f, %f, %f", m_bob->target_point.x, m_bob->target_point.y, m_bob->target_point.z);
		animTcl::OutputMessage("lerping_to_beginning_of_spline: %f", lerping_to_beginning_of_spline);

		//////////////////////////////////////////////////////
		// Temporarily make dtX an eigen vec4 instead.
		dtX_eigen << Ptarget[0] - intermediateEnd[0], Ptarget[1] - intermediateEnd[1], Ptarget[2] - intermediateEnd[2];

		// Jacobian determines how the EndEffector changes based on current angles and the hand point in hand coordinate system
		Jacobian j(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, Eigen::Vector4d{ 0.0, m_bob->L3, 0.0, 1.0 });

		// Find BETA
		BetaSolver beta(j, dtX_eigen);

		// FIND PSEUDO INVERSE OF JACOBIAN BASED ON BETA
		dtThetas = j.jacobian.transpose() * beta.beta;


		// UPDATE THETAS
		m_bob->angles.theta1 = m_bob->angles.theta1 + dtThetas[0];
		m_bob->angles.theta2 = m_bob->angles.theta2 + dtThetas[1];
		m_bob->angles.theta3 = m_bob->angles.theta3 + dtThetas[2];
		m_bob->angles.theta4 = m_bob->angles.theta4 + dtThetas[3];
		m_bob->angles.theta5 = m_bob->angles.theta5 + dtThetas[4];
		m_bob->angles.theta6 = m_bob->angles.theta6 + dtThetas[5];
		m_bob->angles.theta7 = m_bob->angles.theta7 + dtThetas[6];

		// Update LERP scaler
		lerping_to_beginning_of_spline += 0.01;
	}

	/* SECOND OF ALL HANDLE TRAVERSING THE SPLINE */
	if (lerping_to_beginning_of_spline== 1.0 && !drawingPath->controlPoints[cPointID + 1].empty)
	{
		// Update endEffector to be start of spline only once then moving along spline
		if (!lerping_to_beginning_of_spline)
		{
			endEffector << drawingPath->controlPoints[cPointID].point.x, drawingPath->controlPoints[cPointID].point.y, drawingPath->controlPoints[cPointID].point.z, 1.0;
		}

		// Update Ptarget to be next point on spline
		Ptarget << drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.x, 
			drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.y, 
			drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.z, 
			1.0;

		// Make dtX an eigen vec4 instead.
		dtX_eigen << Ptarget - endEffector;


		// Jacobian determines how the EndEffector changes based on current angles and the hand point in hand coordinate system
		Jacobian j(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, Eigen::Vector4d{ 0.0, m_bob->L3, 0.0, 1.0 });


		// Find BETA
		BetaSolver beta(j, dtX_eigen);

		// FIND PSEUDO INVERSE OF JACOBIAN BASED ON BETA
		dtThetas = j.jacobian.transpose() * beta.beta;


		// UPDATE THETAS
		m_bob->angles.theta1 = m_bob->angles.theta1 + dtThetas[0];
		m_bob->angles.theta2 = m_bob->angles.theta2 + dtThetas[1];
		m_bob->angles.theta3 = m_bob->angles.theta3 + dtThetas[2];
		m_bob->angles.theta4 = m_bob->angles.theta4 + dtThetas[3];
		m_bob->angles.theta5 = m_bob->angles.theta5 + dtThetas[4];
		m_bob->angles.theta6 = m_bob->angles.theta6 + dtThetas[5];
		m_bob->angles.theta7 = m_bob->angles.theta7 + dtThetas[6];

		// update endEffector
		endEffector = Ptarget;

		// Update t parameter to walk closer towards the next control point
		t++;
		// update control point id when t==1
		if (t) {
			cPointID++;
			t = 0.00001;
		}
	}




	return 0;
}



void BobDraws::initializePs() {

	// Initialize resting position in degrees
	m_bob->angles = rAngles{ 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0 };
	// Create an initial eigen point for end effector in world coordinates	
	endEffector << 0.0, 0.0, 0.0, 1.0; 

	// Get the series of transformations to multiply end effector by to get its position in world coordinates
	Transformations transform(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3);

	// now end Effector represents Pinitial in world coordinates
	endEffector = transform.matrixTransform * endEffector;
	
	// Ptarget is also in world coordinates representing the beginning of the spline
	Ptarget << drawingPath->controlPoints[0].point.x, drawingPath->controlPoints[0].point.y, drawingPath->controlPoints[0].point.z, 1.0;
	
	///////////////////// VARS FOR TESTING /////////
	m_bob->temp_end_eff = { endEffector[0], endEffector[1], endEffector[2] };
	m_bob->target_point = glm::vec3{ Ptarget[0], Ptarget[1], Ptarget[2] };
	
	/////////////////////////////////////////////

	// Make dtX an eigen vec4 instead.
	dtX_eigen << Ptarget[0] - endEffector[0], Ptarget[1] - endEffector[1], Ptarget[2] - endEffector[2];

	intermediateEnd << dtX_eigen[0], dtX_eigen[1], dtX_eigen[2], 0.0;
	
	
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