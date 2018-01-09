//#include "SolverQP.h"
//#include "RecursionLBFGS.h"
//
//// Define default constructor. 
//LASO::SolverQP::SolverQP()
//{
//
//}
//
//// Main method used to solve QP problem.
//void LASO::SolverQP::solve()
//{
//
//}
//
//// Function that evaluates bVector.
//void LASO::SolverQP::evaluateBVector(uvec &kIteration, vec &gradientX,
//	vec &hEqualityConstraints, mat &jhEqualityJacobian,
//	uvec &noEqualityConstraints,
//	vec &gInequalityConstraints, mat &jgInequalityJacobian,
//	uvec &activeInequalityConstraints, uvec &noActiveInequalityConstraints,
//	mat &sVectors, mat &yVectors,
//	uvec &correctionHistory, uvec &noCorrections,
//	uvec &inactiveVariables, uvec &noInactiveVariables,
//	vec &qVector, vec &inverseHessianScaling, vec &rVector,
//	vec &bVector)
//{
//	// If constrained problem.
//	if (noEqualityConstraints(0) + noActiveInequalityConstraints(0) > 0)
//	{
//		// Resize bVector.
//		bVector.set_size(noEqualityConstraints(0) + noActiveInequalityConstraints(0));
//
//		// If first iteration, take H as identity.
//		// TODO: Take H as scaled identity.
//		if (kIteration(0) < 1)
//		{
//			// Evaluate qVector
//			for (integer i = 0; i < noInactiveVariables(0); ++i)
//			{
//				qVector[inactiveVariables(i)] = gradientX[inactiveVariables(i)];
//			}
//
//			// Evaluate bVector
//			for (integer i = 0; i < noEqualityConstraints(0); ++i)
//			{
//				bVector[i] = hEqualityConstraints[i];
//
//				for (integer j = 0; j < noInactiveVariables(0); ++j)
//				{
//					bVector[i] -= jhEqualityJacobian[i][inactiveVariables(j)] * qVector[inactiveVariables(j)];
//				}
//			}
//			for (integer i = noEqualityConstraints(0); i < noEqualityConstraints(0) + noActiveInequalityConstraints(0); ++i)
//			{
//				bVector[i] = gInequalityConstraints[activeInequalityConstraints[i - noEqualityConstraints(0)]];
//
//				for (integer j = 0; j < noInactiveVariables(0); ++j)
//				{
//					bVector[i] -= jgInequalityJacobian[activeInequalityConstraints[i - noEqualityConstraints(0)]][inactiveVariables(j)] * qVector[inactiveVariables(j)];
//				}
//			}
//		}
//		// Else take H as l-bfgs approximation of the inverse Hessian of the Lagrangian function.
//		else
//		{
//			// Evaluate qVector
//			for (integer i = 0; i < noInactiveVariables(0); ++i)
//			{
//				qVector[inactiveVariables(i)] = gradientX[inactiveVariables(i)];
//			}
//
//			// Evaluate rVector = Hq
//			LASO::RecursionLBFGS recursionLBFGS;
//			recursionLBFGS.evaluateRVector(sVectors, yVectors,
//				qVector, inactiveVariables, noInactiveVariables(0),
//				kIteration, noCorrections, correctionHistory,
//				qVector, inverseHessianScaling(0), rVector);
//
//			// Evaluate bVector
//			for (integer i = 0; i < noEqualityConstraints(0); ++i)
//			{
//				bVector[i] = hEqualityConstraints[i];
//
//				for (integer j = 0; j < noInactiveVariables(0); ++j)
//				{
//					bVector[i] -= jhEqualityJacobian[i][inactiveVariables(j)] * rVector[inactiveVariables(j)];
//				}
//			}
//			for (integer i = noEqualityConstraints(0); i < noEqualityConstraints(0) + noActiveInequalityConstraints(0); ++i)
//			{
//				bVector[i] = gInequalityConstraints[activeInequalityConstraints[i - noEqualityConstraints(0)]];
//
//				for (integer j = 0; j < noInactiveVariables(0); ++j)
//				{
//					bVector[i] -= jgInequalityJacobian[activeInequalityConstraints[i - noEqualityConstraints(0)]][inactiveVariables(j)] * rVector[inactiveVariables(j)];
//				}
//			}
//		}
//	}
//}