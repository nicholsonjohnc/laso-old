//#pragma once
//
//#ifndef LASO_SOLVERQP
//#define LASO_SOLVERQP
//
//#include "common.h"
//#include "armadillo"
//
//using namespace arma;
//
//namespace LASO
//{
//	// Declare SolverQP class. 
//	class SolverQP
//	{
//
//	public:
//
//		// Default constructor.
//		SolverQP();								
//
//		// Main method used to solve QP problem.
//		void solve();
//
//		// Function that evaluates bVector.
//		void evaluateBVector(uvec &kIteration, vec &gradientX,
//			vec &hEqualityConstraints, mat &jhEqualityJacobian, 
//			uvec &noEqualityConstraints,
//			vec &gInequalityConstraints, mat &jgInequalityJacobian,
//			uvec &activeInequalityConstraints, uvec &noActiveInequalityConstraints,
//			mat &sVectors, mat &yVectors,
//			uvec &correctionHistory, uvec &noCorrections,
//			uvec &inactiveVariables, uvec &noInactiveVariables,
//			vec &qVector, vec &inverseHessianScaling, vec &rVector,
//			vec &bVector);
//
//	private:
//
//	};
//
//}
//
//#endif
