#include "BobDraws.h"

BobDraws::BobDraws(const std::string& name, HermiteSpline* drawing, Bob* bob) :
	BaseSimulator(name),
	drawingPath(drawing),
	m_bob(bob), P_endEffector(4), P_target(4), dtThetas(7), Error(4), start(4), end(4)
{											 
}	// BobDraws								  

BobDraws::~BobDraws()						  
{											  
}	// BobDraws::~BobDraws

int BobDraws::step(double time)
{
	/***************************************************************/
	/* FIRST GET TO SPLINE lerp function */
		
	if (lerp_iteration < 100) { 
		
		// Lerp to the start of spline over 25 frames
				
		for (int display = 0; display < 4; display++) // Get to start of spline in 25 seconds
		{
			lerp_iteration++;
			lerp(start, end, 0.01*lerp_iteration);
			converge(start, end, 0.01 * lerp_iteration, 1);
		}

	} else if (!drawingPath->controlPoints[cPointID + 1].empty)

	{
	/***************************************************************/
	/* SECOND TRAVERSE SPLINE */
		
		point_lerp++;
		// START (at first control point)
		if (startDrawingBob)
		{
			//converge to end
			//converge(P_endEffector, end, 1, 3);

			start << end; // start is beginning of spline
			
			end << drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.x,
				drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.y,
				drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.z,
				1.0;
			
			
			t += 0.01;

			// LERP BETWEEN THE CURRENT POINTS
			for (int display = 0; display < 4; display++)
			{
				lerp(start, end, 0.1 * point_lerp);
				converge(start, end, 0.1 * point_lerp, 100);
			}
			startDrawingBob = false;
			return 0;
		} 
						
		if (point_lerp == 10)
		
		{
			if (t)

			{   // END (reaching a control point)
				// Reset and update
				start = end;
				end << drawingPath->controlPoints[cPointID].point.x,
					drawingPath->controlPoints[cPointID].point.y,
					drawingPath->controlPoints[cPointID].point.z, 1.0;
				cPointID++;
				t = 0.01;
			}

			else {
				// MIDDLE (between control points) -->ONLY UPDATE at point_lerp=10
				start = end;
				end << drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.x,
					drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.y,
					drawingPath->getNext(drawingPath->controlPoints[cPointID], drawingPath->controlPoints[cPointID + 1], t).point.z,
					1.0;
				
			}
			t += 0.01;
			point_lerp = 0;

			// LERP BETWEEN THE CURRENT POINTS
			for (int display = 0; display < 4; display++)
			{
				lerp(start, end, 0.1 * point_lerp);
				converge(start, end, 0.1 * point_lerp, 100);
			}
			return 0;
		}
		
	}
	else
	{

		/***************************************************************/
		/* REACHED END OF SPLINE SO RESET TO START OF SPLINE*/


	}

	return 0;
}

void BobDraws::converge(Eigen::Vector4d start, Eigen::Vector4d end, float scalar, int max)
{
	Eigen::Vector4d step;
	/***************************************************************/
	/* STEP 1: find p target by linearly interpolating between start and end by scalar*/
	P_target = (1.0f - scalar) * start + end * scalar;
	

	/***************************************************************/
	/* STEP 2: update nescessary variables*/

	// 2) a. update P_EndEffector
	EndEffectorWorldCoord transform(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, m_bob->z);
	P_endEffector = transform.matrixTransform * Eigen::Vector4d{ 0.0, 0.0, 0.0, 1 };

	// 2) b. update error based on where bob's hand actually ended up
	Error = P_target - P_endEffector; // vec4

	/***************************************************************/
	/* STEP 3: converge to P_target*/
	int max_iterations = 0;

	while (Error.norm() > Epsilon && max_iterations < max)
	{
		// 3) a. Make a step along the error towards the actual target
		step = Error * 0.2f;

		// 3) b. Compute Jacobian based on current angles
		Jacobian j(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, m_bob->z);

		// 3) b. Find BETA based on step
		BetaSolver beta(j, Eigen::Vector3d{ step[0], step[1], step[2] });

		// 3) c. calculate dt Thetas based on Jt * beta
		dtThetas << j.jacobian.transpose() * beta.beta;


		// 3) d. update joint angles
		updateAngles(dtThetas);

		// 3) e. Recalculate actual end effector after the angles have been updated with the new jacobian. 
		EndEffectorWorldCoord transform(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, m_bob->z);
		P_endEffector << transform.matrixTransform * Eigen::Vector4d{ 0.0, 0.0, 0.0, 1 };

		// 3) f. Recalculate error
		Error << P_target - P_endEffector;

		// 3) g. update max iterations to avoid inf loops
		max_iterations++;

	}

}

/* The following function is given two points in space and a scalar. 
   It's purpose is to move the arm between those points in space by a certain scalar*/
void BobDraws::lerp(Eigen::Vector4d start, Eigen::Vector4d end, float scalar)
{
	Eigen::Vector4d step;

	animTcl::OutputMessage("scalar is: %f", scalar);
	glm::vec4 start_print = { start[0], start[1], start[2], start[3] };
	glm::vec4 end_print = { end[0], end[1], end[2], end[3] };
	animTcl::OutputMessage("current start is: %f %f %f", start_print.x, start_print.y, start_print.z, start_print.w);
	animTcl::OutputMessage("current end vector is: %f %f %f %f", end_print.x, end_print.y, end_print.z, end_print.w);


	/***************************************************************/
	/* STEP 1: find p target by linearly interpolating between start and end by scalar*/
	P_target = (1.0f - scalar) * start + end * scalar; 
	

	/***************************************************************/
	/* STEP 2: Update the nescessary variables */

	// 2) a. find the step that would get me to P_target
	step = P_target - P_endEffector; // gives vec4


	/***************************************************************/
	/* STEP 3: Make an actual step */

	// 3) a. Compute Jacobian based on current angles
	Jacobian j(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, m_bob->z);

	// 3) b. Find BETA based on step
	BetaSolver beta(j, Eigen::Vector3d{ step[0], step[1], step[2] });

	// 3) c. calculate dt Thetas based on Jt * beta
	dtThetas = j.jacobian.transpose() * beta.beta;

	// 3) d. update joint angles
	updateAngles(dtThetas);

	/****************************************************************/
	/* STEP 4: update nescessary variables*/

	// 4) a. update P_EndEffector
	EndEffectorWorldCoord transform(m_bob->angles, m_bob->L1, m_bob->L2, m_bob->L3, m_bob->z);
	P_endEffector = transform.matrixTransform * Eigen::Vector4d{ 0.0, 0.0, 0.0, 1 };

	///////TEST///////////////////////////////////
	animTcl::OutputMessage("");
	animTcl::OutputMessage("END OF LERP");
	print_vals_for_testing(step);

	////////////////////////////////////
}

void BobDraws::print_vals_for_testing(Eigen::Vector4d step)
{
	glm::vec3 step_print = { step[0], step[1], step[2] };
	glm::vec3 p_target_print = { P_target[0], P_target[1], P_target[2] };
	glm::vec3 p_end_effector_print = { P_endEffector[0], P_endEffector[1], P_endEffector[2] };
	glm::vec4 error_print = { Error[0], Error[1], Error[2], Error[3] };
	
	animTcl::OutputMessage("print delta Thetas: %f %f %f %f %f %f %f", dtThetas[0], dtThetas[1], dtThetas[2], dtThetas[3], dtThetas[4], dtThetas[5], dtThetas[6]);
	animTcl::OutputMessage("print angles: %f %f %f %f %f %f %f", m_bob->angles.theta1, m_bob->angles.theta2, m_bob->angles.theta3, m_bob->angles.theta4, m_bob->angles.theta5, m_bob->angles.theta6, m_bob->angles.theta7);
	animTcl::OutputMessage("current step is: %f %f %f", step_print.x, step_print.y, step_print.z);
	animTcl::OutputMessage("current P_target is: %f %f %f", p_target_print.x, p_target_print.y, p_target_print.z);
	animTcl::OutputMessage("current P_endEffector is: %f %f %f", p_end_effector_print.x, p_end_effector_print.y, p_end_effector_print.z);
	animTcl::OutputMessage("current error vector is: %f %f %f %f", error_print.x, error_print.y, error_print.z, error_print.w);
	animTcl::OutputMessage("current error size is: %f\n\n", Error.norm());
	animTcl::OutputMessage("");
	animTcl::OutputMessage("");
}

void BobDraws::updateAngles(Eigen::VectorXd newThetas)
{
	// UPDATE THETAS
	m_bob->angles.theta1 = m_bob->angles.theta1 + newThetas[0];
	m_bob->angles.theta2 = m_bob->angles.theta2 + newThetas[1];
	m_bob->angles.theta3 = m_bob->angles.theta3 + newThetas[2];
	m_bob->angles.theta4 = m_bob->angles.theta4 + newThetas[3];
	m_bob->angles.theta5 = m_bob->angles.theta5 + newThetas[4];
	m_bob->angles.theta6 = m_bob->angles.theta6 + newThetas[5];
	m_bob->angles.theta7 = m_bob->angles.theta7 + newThetas[6];
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
	
	// P_target is also in world coordinates representing the beginning of the spline
	P_target << drawingPath->controlPoints[0].point.x, drawingPath->controlPoints[0].point.y, drawingPath->controlPoints[0].point.z, 1.0;
	
	start << P_endEffector;
	end << drawingPath->controlPoints[0].point.x, drawingPath->controlPoints[0].point.y, drawingPath->controlPoints[0].point.z, 1.0;

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