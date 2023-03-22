#include "BetaSolver.h"

#include "BetaSolver.h"

BetaSolver::BetaSolver(Jacobian j, glm::vec3 dtX) 
{
	beta = betaSolver(j, dtX);
}

// Solve for beta
Eigen::Vector3d BetaSolver::betaSolver(Jacobian j, glm::vec3 dtX)
{
	// Make velocity Eigen
	Eigen::Vector3d vel;
	vel << dtX.x, dtX.y, dtX.z ;
	// Make a j jt matrix (j multiplied by its transpose)
	Eigen::Matrix3Xd jjt = j.jacobian * j.jacobian.transpose();
	// Use the eigen properties and functions to solve for beta with Lower Upper triangular
	Eigen::PartialPivLU<Eigen::Matrix3d> lu(jjt);
	Eigen::Vector3d beta = lu.solve(vel);
	return beta;
}



