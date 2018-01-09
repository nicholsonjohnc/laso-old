#pragma once

#ifndef LASO_PROBLEMAL
#define LASO_PROBLEMAL

#include <iostream>
#include <iomanip>
#include <fstream>
#include "IFunctions.h"
#include "Options.h"



namespace LASO
{

	class ProblemAL
	{

	public:

		// Declare public attributes.
		std::ofstream outputFile;					// File output object.
		std::ofstream solutionFile;					// File output object.
		integer nDesignVariables;					// Number of optimization variables.
		IFunctions &functions;			// Reference to object containing optimization problem functions objFun, objGrad, conFun, and conJac.
		double *xDesignVariables_k;					// Optimization variable values at kth iteration.
		vector xDesignVariables_kPlus1;				// Optimization variable values at (k + 1)th iteration.
		vector xDesignVariablesTrial;				// Trial optimization variable values used during line search.
		vector &lbLowerBounds;						// Lower bounds on the optimization variables and general linear and nonlinear constraints.
		vector &ubUpperBounds;						// Upper bounds on the optimization variables and general linear and nonlinear constraints.
		integer mConstraints;						// Number of general linear and nonlinear constraints.
		Options &options;				// Optimization options object containing configuration and algorithmic parameters.
		integer kIteration;							// Current iteration.
		integer kIterationSinceRestart;				// Iterations since restart.
		double *fObjectiveFunction_k;				// Objective function value at kth iteration.
		vector fObjectiveFunction_kPlus1;			// Objective function value at (k + 1)th iteration.
		vector phiLineSearchFunction_k;				// Line search function value.
		double *objectiveGradient_k;					// Objective gradient values at kth iteration.
		vector objectiveGradient_kPlus1;			// Objective gradient values at (k + 1)th iteration.
		double *fConstraintFunctions_k;				// Constraint function values at kth iteration.
		double *jConstraintJacobian_k;				// Constraint Jacobian values at kth iteration.
		integer nofObjEvaluations;					// Number of objective function evaluations.
		integer nofConEvaluations;					// Number of constraint function evaluations.
		integer nojConEvaluations;					// Number of constraint Jacobian evaluations.
		integer nogObjEvaluations;					// Number of objective gradient evaluations.
		integer noMinorIterations;					// Number of minor iterations.
		vectorinteger EqualityConstraintSet;		// Indices of equality constraints.
		vectorinteger InequalityConstraintSetL;		// Indices of lower bound inequality constraints.
		vectorinteger InequalityConstraintSetU;		// Indices of upper bound inequality constraints.
		integer mENoEqualityConstraints;			// Total number of auxiliary equality constraints. 
		integer mINoInequalityConstraintsU;			// Total number of auxiliary upper bound inequality constraints.
		integer mINoInequalityConstraintsL;			// Total number of auxiliary lower bound inequality constraints.
													//vector rObjectiveGradient_kPlus1;			// Reformatted objective gradient values at (k + 1)th iteration.
		vector lagrangianGradient_k;				// Lagrangian gradient values at kth iteration.
		vector lagrangianGradientQP;				// Lagrangian gradient values at kth iteration with Quadratic Programming Lagrange multipliers.
		vector lagrangianGradient_kPlus1;				// Lagrangian gradient values at (k + 1)th iteration.
														//vector rConstraintFunctions_k;				// Reformatted vector of constraints at kth iteration.
														//vector rConstraintFunctions_kPlus1;			// Reformatted vector of constraints at (k + 1)th iteration.
														//matrix jrConstraintJacobian_k;				// Reformatted constraint Jacobian at kth iteration.
														//matrix jrConstraintJacobian_kPlus1;			// Reformatted constraint Jacobian at (k + 1)th iteration.
		vector zSlackVariables_k;					// Slack variable values at kth iteration.
		vector zSlackVariables_kPlus1;				// Slack variable values at (k + 1)th iteration.
		vectorinteger activeConstraintSet_k;		// Indices of active constraints at kth iteration.
		vectorinteger inactiveConstraintSet_k;		// Indices of inactive constraints at kth iteration.
		vectorinteger inactiveVariableSet_k;		// Indices of inactive variables at kth iteration.
		integer mANoActiveConstraints_k;			// Total number of active constraints at kth iteration.
		integer mINoInactiveConstraints_k;			// Total number of inactive constraints at kth iteration.
		integer nINoInactiveVariables_k;			// Total number of inactive variables at kth iteration.
		vector hConstraintFunctions_k;				// Auxiliary vector of equality constraints at kth iteration.
		vector hConstraintFunctions_kPlus1;				// Auxiliary vector of equality constraints at (k + 1)th iteration.
		vector gConstraintFunctions_k;				// Auxiliary vector of inequality constraints at kth iteration.
		vector gConstraintFunctions_kPlus1;				// Auxiliary vector of inequality constraints at (k + 1)th iteration.
		matrix jhConstraintJacobian_k;				// Auxiliary equality constraint Jacobian values at kth iteration.
		matrix jhConstraintJacobian_kPlus1;			// Auxiliary equality constraint Jacobian values at (k + 1)th iteration.
		matrix jgConstraintJacobian_k;				// Auxiliary inequality constraint Jacobian values at kth iteration.
		matrix jgConstraintJacobian_kPlus1;				// Auxiliary inequality constraint Jacobian values at (k + 1)th iteration.
		vector constraintViolations_k;				// Vector of constraint violations at kth iteration.
		vector maxConstraintViolation_k;			// Maximum constraint violation at kth iteration.
		vector optimalityViolations_k;				// Vector of optimality violations at kth iteration.
		vector maxOptimalityViolation_k;			// Maximum optimality violation at kth iteration.
		vector maxMinorOptimalityViolation_k;		// Maximum minor optimality violation at kth iteration.
		vector lambdaLagrangeMultipliersL_k;		// Lower bound inequality constraint Lagrange multiplier values at kth iteration.
		vector lambdaLagrangeMultipliersU_k;		// Upper bound inequality constraint Lagrange multiplier values at kth iteration.
		vector lambdaLagrangeMultipliers_k;			// Auxiliary inequality constraint Lagrange multiplier values at kth iteration.
		vector lambdaLagrangeMultipliers_kPlus1;	// Auxiliary inequality constraint Lagrange multiplier values at (k + 1)th iteration.
		vector lambdaLagrangeMultipliersQP;			// Auxiliary inequality constraint quadratic programming Lagrange multiplier values.
		vector muLagrangeMultipliers_k;				// Auxiliary equality constraint Lagrange multiplier values at kth iteration.
		vector muLagrangeMultipliers_kPlus1;		// Auxiliary equality constraint Lagrange multiplier values at (k + 1)th iteration.
		vector muLagrangeMultipliersQP;				// Auxiliary equality constraint quadratic programming Lagrange multiplier values.
		vector dxSearchDirection_k;					// Search direction with respect to the design variables.
		vector dzSearchDirection_k;					// Search direction with respect to the slack variables.
		vector dmuSearchDirection_k;				// Search direction with respect to the auxiliary equality constraint Lagrange multipliers.
		vector dlambdaSearchDirection_k;			// Search direction with respect to the auxiliary inequality constraint Lagrange multipliers.
		vector phPenalties_k;						// Penalty parameters for auxiliary equality constraints.
													//vector phPenalties_kPlus1;					// Penalty parameters for auxiliary equality constraints at (k + 1)th iteration.
		vector pgPenalties_k;						// Penalty parameters for auxiliary inequality constraints.
		vector augmentedLagrangianGradientX_k;		// Augmented Lagrangian gradient with respect to design variables at kth iteration.
		vector augmentedLagrangianGradientXQP;		// Augmented Lagrangian gradient with respect to design variables at kth iteration with Quadratic Programming Lagrange multipliers.
		vector augmentedLagrangianGradientX_kPlus1;	// Augmented Lagrangian gradient with respect to design variables at (k + 1)th iteration.
		vector augmentedLagrangianGradientZ_k;		// Augmented Lagrangian gradient with respect to slack variables at kth iteration.
		vector augmentedLagrangianGradientZQP;		// Augmented Lagrangian gradient with respect to slack variables at kth iteration with Quadratic Programming Lagrange multipliers.
		vector augmentedLagrangianGradientZ_kPlus1;	// Augmented Lagrangian gradient with respect to slack variables at (k + 1)th iteration.
		vector augmentedLagrangianGradientMu_k;		// Augmented Lagrangian gradient with respect to the auxiliary equality constraint Lagrange multipliers at kth iteration.
		vector augmentedLagrangianGradientMuQP;		// Augmented Lagrangian gradient with respect to the auxiliary equality constraint Lagrange multipliers at kth iteration with Quadratic Programming Lagrange multipliers.
		vector augmentedLagrangianGradientMu_kPlus1;// Augmented Lagrangian gradient with respect to the auxiliary equality constraint Lagrange multipliers at (k + 1)th iteration.
		vector augmentedLagrangianGradientLambda_k;		// Augmented Lagrangian gradient with respect to the auxiliary inequality constraint Lagrange multipliers at kth iteration.
		vector augmentedLagrangianGradientLambdaQP;		// Augmented Lagrangian gradient with respect to the auxiliary inequality constraint Lagrange multipliers at kth iteration with Quadratic Programming Lagrange multipliers.
		vector augmentedLagrangianGradientLambda_kPlus1;// Augmented Lagrangian gradient with respect to the auxiliary inequality constraint Lagrange multipliers at (k + 1)th iteration.
		vector phiPrime0_k;							// Line search function gradient at kth iteration.
		doublereal rhoFactor;						// Line search factor, rho, used in sufficient decrease condition.
		doublereal betaFactor;						// Line search factor, beta, used in curvature condition.
		doublereal etaFactor;						// Line search factor, eta, used to decrease step size.
		doublereal sigmaOneFactor;					// Search direction factor, sigmaOneFactor, used to determine if direction of descent.
		doublereal sigmaTwoFactor;					// Search direction factor, sigmaTwoFactor, used to determine if direction of descent.
		vector alphaStepSize_k;					// Step size value at kth iteration.
		vector inverseHessianScaling_k;				// Inverse Hessian scale factor at kth iteration.
		string lineSearchProcedure_k;				// Procedure used to calculate accepted step size at kth iteration.
		vectorcharacter code;						// Vector of code characters indicating what happened during the course of the major iteration.

													// Augmented Lagrangian specific variables.
		vector qVector;								// Working vector for l-bfgs design variable and slack variable search direction calculations.
		vector rVector;								// Working vector for l-bfgs design variable and slack variable search direction calculations.
		vectorinteger correctionHistory;			// Vector used to keep track of what correction corresponds to what iteration.
		matrix sVectors;							// S correction vectors for l-bfgs design variable search direction calculations. 
		matrix yVectors;							// Y correction vectors for l-bfgs design variable search direction calculations. 
		doublereal directionMagnitude;				// Working variable representing the search direction magnitude.
		doublereal gradientMagnitude;				// Working variable representing the gradient magnitude.
		doublereal directionDotGradient;			// Working variable representing dot product of search direction and gradient.
		vector bVector_k;							// Vector used by Lagrange multiplier sub-problem.
		matrix BMatrix_k;							// Matrix used by Lagrange multiplier sub-problem.
		vector lagrangeMultiplierSearchDirection;	// Vector used by Lagrange multiplier sub-problem.
		vector cVector_k;							// Vector used by penalty parameter sub-problem.
		vector CMatrix_k;							// Vector used by penalty parameter sub-problem.
		vector pSolution;							// Penalty parameter sub-problem solution vector.

		//highresolutiontime startTime;
		//highresolutiontime intermediateTime1;
		//highresolutiontime intermediateTime2;
		//highresolutiontime finishTime;
		//doublereal outputSeconds;
		//doublereal objectiveSeconds;
		//doublereal gradientSeconds;
		//doublereal constraintSeconds;
		//doublereal jacobianSeconds;
		//doublereal lasoSeconds;

		ProblemAL(integer n, IFunctions &funcs, double *x, vector &lb, vector &ub, integer m, Options &ops, double *fObj, double *gObj, double *fCon, double *gCon);		// Default constructor.
		~ProblemAL();		// Default destructor.

	private:

		// Declare private attributes.

	};

}

#endif