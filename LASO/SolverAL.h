#pragma once

#ifndef LASO_SOLVERAL
#define LASO_SOLVERAL

#include "ProblemAL.h"

namespace LASO
{
	// Declare SolverAL class. 
	class SolverAL
	{

	public:

		// Declare public attributes.
		//SolverAL *solver;

		SolverAL();								// Default constructor.

		void solve(ProblemAL &problem);			// Main function used to solve an optimization problem.	

		// Function used to solve QP sub-problem.
		void solveQP(ProblemAL &problem);

		bool removeConstraint(ProblemAL &problem);

		bool addConstraint(ProblemAL &problem);

		bool stoppingCriteriaQP(ProblemAL &problem);

		// Function that ensures all variables are on or within their bounds.
		void applyProjectionOperator(double *variables,
			integer noVariables, vector &lowerBound, vector &upperBound);

		// Function that ensures all variables are on or within constant bounds.
		void applyProjectionOperatorConstantBounds(vector &variables,
			integer noVariables, doublereal lowerBound, doublereal upperBound);

		// Function that evaluates the auxiliary vectors of constraints, h(x) and g(x).
		void evaluateAuxiliaryConstraints(double *xDesignVariables,
			vector &hConstraintFunctions, vector &gConstraintFunctions, ProblemAL &problem);

		// Function that evaluates zSlackVariables given gConstraintFunctions.
		void evaluateSlackVariables(vector &gConstraintFunctions,
			vector &lambdaLagrangeMultipliers, vector &pgPenalties,
			vector &zSlackVariables, ProblemAL &problem);

		// Function that evaluates the gradient of the objective function with respect to the design variables.
		void evaluateObjectiveGradient(double *xDesignVariables,
			double *objectiveGradient, ProblemAL &problem);

		// Function that evaluates the auxiliary constraint Jacobians, Jh(x) and Jg(x).
		void evaluateAuxiliaryJacobians(double *xDesignVariables,
			matrix &jhConstraintJacobian, matrix &jgConstraintJacobian, ProblemAL &problem);

		// Function that calculates the maximum constraint violation, Fbar.
		void calculateMaxFeasibilityViolation(vector &hConstraintFunctions,
			vector &gConstraintFunctions, vector &lambdaLagrangeMultipliers,
			vector &maxConstraintViolation, ProblemAL &problem);

		// Function that evaluates the gradient of the Lagrangian function with respect to the design variables.
		void evaluateLagrangianGradient(double *objectiveGradient,
			matrix &jhConstraintJacobian, matrix &jgConstraintJacobian,
			vector &muLagrangeMultipliers, vector &lambdaLagrangeMultipliers,
			vector &lagrangianGradient, ProblemAL &problem);

		// Function that calculates the maximum optimality violation, Obar.
		void calculateMaxOptimalityViolation(vector &lagrangianGradient,
			vectorinteger &inactiveVariableSet, integer noInactiveVariables,
			vector &maxOptimalityViolation, ProblemAL &problem);

		// Function that returns true if current point is feasible, false otherwise.
		boolean testFeasible(vector &maxConstraintViolation, ProblemAL &problem);

		// Function that returns true if current point is optimal, false otherwise.
		boolean testOptimal(vector &maxOptimalityViolation, ProblemAL &problem);

		// Function that returns true if maximum number of iterations is exceeded, false otherwise.
		boolean testIterations(integer kIteration, ProblemAL &problem);

		// Function that returns true if stopping criteria satisfied, false otherwise.
		boolean stoppingCriteria(vector &maxConstraintViolation, vector &maxOptimalityViolation,
			integer kIteration, ProblemAL &problem);

		// Evaluate the gradient of the augmented Lagrangian function with respect to the design variables, slack variables, and Lagrange multipliers.
		void evaluateAugmentedLagrangianGradients(double *objectiveGradient,
			vector &hConstraintFunctions, vector &gConstraintFunctions,
			matrix &jhConstraintJacobian, matrix &jgConstraintJacobian,
			vector &muLagrangeMultipliers, vector &lambdaLagrangeMultipliers,
			vector &phPenalties, vector &pgPenalties, vector &zSlackVariables,
			vector &augmentedLagrangianGradientX, vector &augmentedLagrangianGradientZ,
			vector &augmentedLagrangianGradientMu, vector &augmentedLagrangianGradientLambda,
			ProblemAL &problem);

		// Function that identifies active / inactive set.
		void identifyActiveInactiveVariableSet(double *xDesignVariables,
			vector &gradientX, vector &dxSearchDirection,
			vector &lambdaLagrangeMultipliersL, vector &lambdaLagrangeMultipliersU,
			vectorinteger &inactiveVariableSet, ProblemAL &problem);

		// Function that estimates active / inactive constraint set.
		void estimateActiveInactiveConstraintSet(vector &zSlackVariables,
			vector &gradientZ, vector &dzSearchDirection, vector &dlambdaSearchDirection,
			vector &lambdaLagrangeMultipliersL, vector &lambdaLagrangeMultipliersU, vector &lambdaLagrangeMultipliers,
			vectorinteger &activeConstraintSet, vectorinteger &inactiveConstraintSet,
			ProblemAL &problem);

		// Function that outputs optimization problem status.
		void outputProblemStatus(ProblemAL &problem);

		// Function that outputs optimization problem solution.
		void outputProblemSolution(ProblemAL &problem);

		// Function that evaluates bVector, used by Lagrange multiplier sub-problem.
		void evaluateBVector(integer kIteration,
			vector &hConstraintFunctions, vector &gConstraintFunctions,
			matrix &jhConstraintJacobian, matrix &jgConstraintJacobian,
			vector &gradientX, matrix &sVectors, matrix &yVectors,
			vectorinteger &variables, integer noVariables,
			vectorinteger &activeConstraints, integer noActiveConstraints,
			vectorinteger &correctionHistory, integer noCorrections,
			vector &bVector, ProblemAL &problem);

		// Function that evaluates BMatrix, used by Lagrange multiplier sub-problem.
		void evaluateBMatrix(integer kIteration,
			matrix &jhConstraintJacobian, matrix &jgConstraintJacobian,
			matrix &sVectors, matrix &yVectors,
			vectorinteger &variables, integer noVariables,
			vectorinteger &activeConstraints, integer noActiveConstraints,
			vectorinteger &correctionHistory, integer noCorrections,
			matrix &BMatrix, ProblemAL &problem);

		// Function that evaluates the Lagrange multiplier search direction.
		void evaluateLagrangeMultiplierSearchDirection(matrix &BMatrix, vector &bVector,
			vectorinteger &activeConstraints, integer noActiveConstraints,
			vector &dmuSearchDirection, vector &dlambdaSearchDirection, ProblemAL &problem);

		// Function that evaluates the QP Lagrange multipliers.
		void evaluateLagrangeMultipliersQP(
			vector &muLagrangeMultipliers, vector &lambdaLagrangeMultipliers,
			vector &dmuSearchDirection, vector &dlambdaSearchDirection,
			vector &muLagrangeMultipliersQP, vector &lambdaLagrangeMultipliersQP,
			ProblemAL &problem);

		// Function that evaluates the inactive set l-bfgs design variable search direction, dx.
		void evaluateInactiveDesignVariableSearchDirection(integer kIteration,
			vector &gradient, matrix &sVectors, matrix &yVectors,
			vectorinteger &variables, integer noVariables,
			vectorinteger &correctionHistory, integer noCorrections,
			vector &dxSearchDirection, ProblemAL &problem);

		// Calculate slack variable search direction.
		void evaluateInactiveSlackVariableSearchDirection(vector &dxSearchDirection,
			vector &gConstraintFunctions, matrix &jgConstraintJacobian,
			vectorinteger &inactiveConstraintSet, integer noInactiveConstraints,
			vectorinteger &variables, integer noVariables,
			vector &zSlackVariables, vector &dzSearchDirection, ProblemAL &problem);

		// Function that evaluates cVector, used by penalty parameter sub-problem.
		void evaluateCVector(vector &lagrangianGradientQP,
			vector &lagrangianGradient, vector &dxSearchDirection,
			vector &lambdaLagrangeMultipliers, vector &dzSearchDirection,
			vector &hConstraintFunctions, vector &dmuSearchDirection,
			vector &gConstraintFunctions, vector &zSlackVariables,
			vector &dlambdaSearchDirection, vector &cVector, ProblemAL &problem);

		// Function that evaluates CMatrix, used by penalty parameter sub-problem.
		void evaluateCMatrix(vector &dxSearchDirection, vector &dzSearchDirection,
			matrix &jhConstraintJacobian, vector &hConstraintFunctions,
			matrix &jgConstraintJacobian, vector &gConstraintFunctions, vector &zSlackVariables,
			vector &CMatrix, ProblemAL &problem);

		// Function that updates equality and inequality constraint penalty parameter vectors.
		void updatePenaltyParameterVectors(vector &CMatrix, vector &cVector, vector &phiPrime0,
			vector &lagrangianGradientQP, vector &dxSearchDirection,
			vector &phPenalties, vector &pgPenalties, ProblemAL &problem);

		// Function that ensures calculated search direction is that of descent.
		void ensureInactiveSearchDirectionThatOfDescent(vector &direction,
			vector &gradient, vectorinteger &components, integer noComponents,
			vectorinteger &correctionHistory, ProblemAL &problem);

		// Initialize step size.
		void initializeStepSize(vector &alpha, 
			vector &zSlackVariables, vector &dzSearchDirection,
			vector &lambdaLagrangeMultipliers, vector &dlambdaSearchDirection,
			ProblemAL &problem);

		// Identify step size.
		void identifyStepSize(vector &alpha, integer kIterationSinceRestart,
			double *xDesignVariables, vector &zSlackVariables,
			vector &muLagrangeMultipliers, vector &lambdaLagrangeMultipliers,
			vector &dxSearchDirection, vector &dzSearchDirection,
			vector &dmuSearchDirection, vector &dlambdaSearchDirection,
			vector &phPenalties, vector &pgPenalties,
			double *fObjectiveFunctionCurrent,
			vector &hConstraintFunctionsCurrent, vector &gConstraintFunctionsCurrent,
			vector &phiPrime0,
			vector &xDesignVariablesTrial, vector &zSlackVariablesTrial,
			vector &muLagrangeMultipliersTrial, vector &lambdaLagrangeMultipliersTrial,
			vector &fObjectiveFunctionTrial,
			vector &hConstraintFunctionsTrial, vector &gConstraintFunctionsTrial,
			ProblemAL &problem);

		// Function that decreases step size.
		doublereal decreaseStepSize(integer kLineSearchIteration,
			vector &alphaValues, vector &phiValues, vector &phiPrime0, doublereal phi0,
			vectorboolean &skipQI, ProblemAL &problem);

		// Function that decreases step size using backtracking.
		doublereal decreaseStepSizeSBT(integer kLineSearchIteration,
			vector &alphaValues, ProblemAL &problem);

		// Function that decreases step size using quadratic interpolation 1.
		doublereal decreaseStepSizeQI1(integer kLineSearchIteration,
			vector &alphaValues, vector &phiValues, vector &phiPrime0, doublereal phi0,
			ProblemAL &problem);

		// Function that decreases step size using quadratic interpolation 2.
		doublereal decreaseStepSizeQI2(integer kLineSearchIteration,
			vector &alphaValues, vector &phiValues, doublereal phi0, ProblemAL &problem);

		// Function that decreases step size using cubic interpolation 1.
		doublereal decreaseStepSizeCI1(integer kLineSearchIteration,
			vector &alphaValues, vector &phiValues, vector &phiPrime0, doublereal phi0,
			ProblemAL &problem);

		// Function that decreases step size using cubic interpolation 2.
		doublereal decreaseStepSizeCI2(integer kLineSearchIteration,
			vector &alphaValues, vector &phiValues, doublereal phi0,
			ProblemAL &problem);

		// Function that returns line search function value for arbitrary step size.
		doublereal returnLineSearchFunction(doublereal stepSize,
			double *xDesignVariables, vector &zSlackVariables,
			vector &muLagrangeMultipliers, vector &lambdaLagrangeMultipliers,
			vector &dxSearchDirection, vector &dzSearchDirection,
			vector &dmuSearchDirection, vector &dlambdaSearchDirection,
			vector &phPenalties, vector &pgPenalties,
			double *xDesignVariablesTrial, vector &zSlackVariablesTrial,
			vector &muLagrangeMultipliersTrial, vector &lambdaLagrangeMultipliersTrial,
			boolean evaluateFunctions, double *fObjectiveFunctionTrial,
			vector &hConstraintFunctionsTrial, vector &gConstraintFunctionsTrial,
			ProblemAL &problem);

		// Function that returns augmented Lagrangian function value.
		doublereal returnAugmentedLagrangianFunction(
			double *xDesignVariablesTrial, vector &zSlackVariablesTrial,
			vector &muLagrangeMultipliersTrial, vector &lambdaLagrangeMultipliersTrial,
			vector &phPenalties, vector &pgPenalties,
			boolean evaluateFunctions, double *fObjectiveFunctionTrial,
			vector &hConstraintFunctionsTrial, vector &gConstraintFunctionsTrial,
			ProblemAL &problem);

		// Function that returns the line search function gradient value for step size of 0.
		void evaluatePhiPrime0LineSearchFunctionGradient(
			vector &gradientX, vector &dxSearchDirection,
			vector &gradientZ, vector &dzSearchDirection,
			vector &gradientMu, vector &dmuSearchDirection,
			vector &gradientLambda, vector &dlambdaSearchDirection,
			vector &phiPrime0, ProblemAL &problem);

		// Function that returns true if sufficient decrease condition, i.e. Armijo's rule, is satisfied.  False otherwise.  
		boolean testSufficientDecreaseCondition(doublereal phiLineSearchFunction,
			doublereal phi0LineSearchFunction, doublereal alphaStepSize,
			vector &phiPrime0, ProblemAL &problem);

		// Function that returns true if modified curvature condition is satisfied.  False otherwise.  
		boolean testCurvatureCondition(doublereal phiLineSearchFunction,
			doublereal phi0LineSearchFunction, doublereal alphaStepSize,
			doublereal phiPrime0LineSearchGradient, ProblemAL &problem);

		// Update s and y correction vectors.
		void updateCorrectionVectors(double *xDesignVariables_k, vector &xDesignVariables_kPlus1,
			vector &gradientX_k, vector &gradientX_kPlus1,
			vectorinteger &variables, integer noVariables,
			vectorinteger &correctionHistory, integer noCorrections,
			matrix &sVectors, matrix &yVectors,
			ProblemAL &problem);

		// Set variables for next iteration.
		void setVariablesForNextIteration(
			double *fObjectiveFunction_k, vector &fObjectiveFunction_kPlus1,
			double *xDesignVariables_k, vector &xDesignVariables_kPlus1,
			vector &zSlackVariables_k, vector &zSlackVariables_kPlus1,
			vector &muLagrangeMultipliers_k, vector &muLagrangeMultipliers_kPlus1,
			vector &lambdaLagrangeMultipliers_k, vector &lambdaLagrangeMultipliers_kPlus1,
			vector &hConstraintFunctions_k, vector &hConstraintFunctions_kPlus1,
			vector &gConstraintFunctions_k, vector &gConstraintFunctions_kPlus1,
			double *objectiveGradient_k, vector &objectiveGradient_kPlus1,
			matrix &jhConstraintJacobian_k, matrix &jhConstraintJacobian_kPlus1,
			matrix &jgConstraintJacobian_k, matrix &jgConstraintJacobian_kPlus1,
			vector &augmentedLagrangianGradientX_k, vector &augmentedLagrangianGradientX_kPlus1,
			vector &augmentedLagrangianGradientZ_k, vector &augmentedLagrangianGradientZ_kPlus1,
			vector &augmentedLagrangianGradientMu_k, vector &augmentedLagrangianGradientMu_kPlus1,
			vector &augmentedLagrangianGradientLambda_k, vector &augmentedLagrangianGradientLambda_kPlus1,
			vector &lagrangianGradient_k, vector &lagrangianGradient_kPlus1,
			ProblemAL &problem);

		void setSearchDirectionToNegAVector(vector &dxSearchDirection, double *aVector, vectorinteger &components, integer noComponents, doublereal scaling, ProblemAL &problem);	// Set dxSearchDirection = -gradient.

	private:

		// Declare private attributes.

	};

}

#endif