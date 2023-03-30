#include "BobDraws.h"

BobDraws::BobDraws(const std::string& name, HermiteSpline* drawing, Bob* bob) :
	BaseSimulator(name),
	drawingPath(drawing),
	m_bob(bob), P_endEffector(4), P_target(4), dtX_eigen(3), endEffector(4), ErrorEndEffector(4), dtThetas(7), ErrorDtThetas(7), end(4)
{											 
}	// BobDraws								  

BobDraws::~BobDraws()						  
{											  
}	// BobDraws::~BobDraws

int BobDraws::step(double time)
{
	////////////TEST JACOBIAN //////////////////////
	static double i = 0.1;
	

	if (i < 0.2) {
		//update variables
		end << drawingPath->controlPoints[0].point.x, drawingPath->controlPoints[0].point.y, drawingPath->controlPoints[0].point.z, 1.0;
		dtX_eigen << end[0] - P_endEffector[0], end[1] - P_endEffector[1], end[2] - P_endEffector[2];
		dtX_eigen << dtX_eigen * i;
		P_target << P_endEffector + Eigen::Vector4d{ dtX_eigen[0], dtX_eigen[0], dtX_eigen[0], 0.0 };

		glm::vec4 end_print = { end[0], end[1], end[2], end[3] };
		glm::vec4 P_target_print = { P_target[0], P_target[1], P_target[2], P_target[3] };
		glm::vec4 P_endEffector_print = { P_endEffector[0], P_endEffector[1], P_endEffector[2], P_endEffector[3] };
		glm::vec3 dtX_print = { dtX_eigen[0], dtX_eigen[1], dtX_eigen[2] };

		animTcl::OutputMessage("end: %f, %f, %f", end_print.x, end_print.y, end_print.z);
		animTcl::OutputMessage("P_target: %f, %f, %f", P_target_print.x, P_target_print.y, P_target_print.z);
		animTcl::OutputMessage("P_endEffector: %f, %f, %f", P_endEffector_print.x, P_endEffector_print.y, P_endEffector_print.z);
		animTcl::OutputMessage("dtX: %f, %f, %f", dtX_print.x, dtX_print.y, dtX_print.z);


		// Update error dt thetas to zero
		ErrorDtThetas *= 0.0;
		ErrorEndEffector << endEffector;
		// ACCOUNT FOR ERROR
		while ((P_target  - ErrorEndEffector).norm() > Error) {
			// Get rid of Error dt thetas and try again
			m_bob->angles.theta1 = m_bob->angles.theta1 - ErrorDtThetas[0];
			m_bob->angles.theta2 = m_bob->angles.theta2 - ErrorDtThetas[1];
			m_bob->angles.theta3 = m_bob->angles.theta3 - ErrorDtThetas[2];
			m_bob->angles.theta4 = m_bob->angles.theta4 - ErrorDtThetas[3];
			m_bob->angles.theta5 = m_bob->angles.theta5 - ErrorDtThetas[4];
			m_bob->angles.theta6 = m_bob->angles.theta6 - ErrorDtThetas[5];
			m_bob->angles.theta7 = m_bob->angles.theta7 - ErrorDtThetas[6];
			//make Error end effector go back
			ErrorEndEffector << endEffector;
			//P_target = P_endEffector + Eigen::Vector4d{ -0.03, 0.1, 0.08, 0.0 };
			

			Jacobian j(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, m_bob->z);

			// Find BETA
			BetaSolver beta(j, dtX_eigen);

			// FIND PSEUDO INVERSE OF JACOBIAN BASED ON BETA
			ErrorDtThetas = j.jacobian.transpose() * beta.beta;


			// UPDATE THETAS
			m_bob->angles.theta1 = m_bob->angles.theta1 + ErrorDtThetas[0];
			m_bob->angles.theta2 = m_bob->angles.theta2 + ErrorDtThetas[1];
			m_bob->angles.theta3 = m_bob->angles.theta3 + ErrorDtThetas[2];
			m_bob->angles.theta4 = m_bob->angles.theta4 + ErrorDtThetas[3];
			m_bob->angles.theta5 = m_bob->angles.theta5 + ErrorDtThetas[4];
			m_bob->angles.theta6 = m_bob->angles.theta6 + ErrorDtThetas[5];
			m_bob->angles.theta7 = m_bob->angles.theta7 + ErrorDtThetas[6];

			EndEffectorWorldCoord transform(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, m_bob->z);

			// update the real endEffector based on the calculated trouble shot angles
			ErrorEndEffector = transform.matrixTransform * endEffector;

			///////////////////// VARS FOR TESTING /////////
			m_bob->temp_end_eff = { P_endEffector[0], P_endEffector[1], P_endEffector[2] };
			m_bob->target_point = glm::vec3{ P_target[0], P_target[1], P_target[2] };

			/////////////////////////////////////////////
			break;
		}
		
		i+= 0.1;
	}
	
	////////////////////////////////////////////////
	///* FIRST OF ALL HANDLE GETTING TO THE SPLINE */
	//if (lerping_to_beginning_of_spline < 1.0)
	//{
	//	m_bob->target_point = { intermediateEnd[0], intermediateEnd[1], intermediateEnd[2] };

	//	animTcl::OutputMessage("intermediateEnd before scaling is as vec3 is: %f, %f, %f", m_bob->temp_end_eff.x, m_bob->temp_end_eff.y, m_bob->temp_end_eff.z);

	//	// update intermediateEnd to be a scalar of P_target.
	//	intermediateEnd << dtX_eigen[0] * lerping_to_beginning_of_spline, dtX_eigen[1] * lerping_to_beginning_of_spline, dtX_eigen[2] * lerping_to_beginning_of_spline, 0.0;

	//	//////////////////////////////// TESTING //////////////
	//	m_bob->target_point = { intermediateEnd[0], intermediateEnd[1], intermediateEnd[2] };
	//	
	//	animTcl::OutputMessage("intermediateEnd after scaling as vec3 is: %f, %f, %f", m_bob->target_point.x, m_bob->target_point.y, m_bob->target_point.z);
	//	animTcl::OutputMessage("lerping_to_beginning_of_spline: %f", lerping_to_beginning_of_spline);

	//	//////////////////////////////////////////////////////
	//	// Temporarily make dtX an eigen vec4 instead.
	//	dtX_eigen << P_target[0] - intermediateEnd[0], P_target[1] - intermediateEnd[1], P_target[2] - intermediateEnd[2];

	//	// Jacobian determines how the P_endEffector changes based on current angles and the hand point in hand coordinate system
		//Jacobian j(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, m_bob->z);

		//// Find BETA
		//BetaSolver beta(j, dtX_eigen);

		//// FIND PSEUDO INVERSE OF JACOBIAN BASED ON BETA
		//dtThetas = j.jacobian.transpose() * beta.beta;


		//// UPDATE THETAS
		//m_bob->angles.theta1 = m_bob->angles.theta1 + dtThetas[0];
		//m_bob->angles.theta2 = m_bob->angles.theta2 + dtThetas[1];
		//m_bob->angles.theta3 = m_bob->angles.theta3 + dtThetas[2];
		//m_bob->angles.theta4 = m_bob->angles.theta4 + dtThetas[3];
		//m_bob->angles.theta5 = m_bob->angles.theta5 + dtThetas[4];
		//m_bob->angles.theta6 = m_bob->angles.theta6 + dtThetas[5];
		//m_bob->angles.theta7 = m_bob->angles.theta7 + dtThetas[6];

	//	// Update LERP scaler
	//	lerping_to_beginning_of_spline += 0.01;
	//}

	///* SECOND OF ALL HANDLE TRAVERSING THE SPLINE */
	//if (lerping_to_beginning_of_spline== 1.0 && !drawingPath->controlPoints[cPointID + 1].empty)
	//{
	//	// Update P_endEffector to be start of spline only once then moving along spline
	//	if (!lerping_to_beginning_of_spline)
	//	{
	//		P_endEffector << drawingPath->controlPoints[cPointID].point.x, drawingPath->controlPoints[cPointID].point.y, drawingPath->controlPoints[cPointID].point.z, 1.0;
	//	}

	//	// Update P_target to be next point on spline
	//	P_target << drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.x, 
	//		drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.y, 
	//		drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.z, 
	//		1.0;

	//	// Make dtX an eigen vec4 instead.
	//	dtX_eigen << P_target - P_endEffector;


	//	// Jacobian determines how the P_endEffector changes based on current angles and the hand point in hand coordinate system
	//	Jacobian j(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, m_bob->z);


	//	// Find BETA
	//	BetaSolver beta(j, dtX_eigen);

	//	// FIND PSEUDO INVERSE OF JACOBIAN BASED ON BETA
	//	dtThetas = j.jacobian.transpose() * beta.beta;


	//	// UPDATE THETAS
	//	m_bob->angles.theta1 = m_bob->angles.theta1 + dtThetas[0];
	//	m_bob->angles.theta2 = m_bob->angles.theta2 + dtThetas[1];
	//	m_bob->angles.theta3 = m_bob->angles.theta3 + dtThetas[2];
	//	m_bob->angles.theta4 = m_bob->angles.theta4 + dtThetas[3];
	//	m_bob->angles.theta5 = m_bob->angles.theta5 + dtThetas[4];
	//	m_bob->angles.theta6 = m_bob->angles.theta6 + dtThetas[5];
	//	m_bob->angles.theta7 = m_bob->angles.theta7 + dtThetas[6];

	//	// update P_endEffector
	//	P_endEffector = P_target;

	//	// Update t parameter to walk closer towards the next control point
	//	t++;
	//	// update control point id when t==1
	//	if (t) {
	//		cPointID++;
	//		t = 0.00001;
	//	}
	//}




	return 0;
}



void BobDraws::initializePs() {

	// Initialize resting position in degrees
	m_bob->angles = rAngles{ 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0, 10.0 * 3.141592653589 / 180.0 };
	// Create an initial eigen point for end effector in world coordinates	
	P_endEffector << 0.0, 0.0, 0.0, 1.0; 

	// Get the series of transformations to multiply end effector by to get its position in world coordinates
	EndEffectorWorldCoord transform(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, m_bob->z);

	// now end Effector represents Pinitial in world coordinates. In the beginning P_endEffector == endEffector
	P_endEffector = transform.matrixTransform * P_endEffector;
	endEffector << P_endEffector;
	// P_target is also in world coordinates representing the beginning of the spline
	P_target << drawingPath->controlPoints[0].point.x, drawingPath->controlPoints[0].point.y, drawingPath->controlPoints[0].point.z, 1.0;
	
	///////////////////// VARS FOR TESTING /////////
	m_bob->temp_end_eff = { P_endEffector[0], P_endEffector[1], P_endEffector[2] };
	m_bob->target_point = glm::vec3{ P_target[0], P_target[1], P_target[2] };
	
	/////////////////////////////////////////////

	
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