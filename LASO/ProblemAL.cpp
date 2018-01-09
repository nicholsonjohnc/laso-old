#include <math.h>
#include "ProblemAL.h"

// Define default constructor for ProblemAL class. 
LASO::ProblemAL::ProblemAL(integer n, IFunctions &funcs, double *x, vector &lb, vector &ub, integer m, Options &ops, double *fObj, double *gObj, double *fCon, double *gCon) :
	functions(funcs),
	//xDesignVariables_k(x),
	lbLowerBounds(lb),
	ubUpperBounds(ub),
	options(ops)
{

	if (options.output)
	{
		outputFile.open("OUTPUT.txt");
		solutionFile.open("SOLUTION.txt");
	}

	nDesignVariables = n;														// Number of optimization variables.
	xDesignVariables_k = x;																			//xDesignVariables_k = x;														// Optimization variable values.
	xDesignVariables_kPlus1.resize(nDesignVariables);							// Optimization variable values at (k + 1)th iteration.
	xDesignVariablesTrial.resize(nDesignVariables);								// Trial optimization variable values used during line search.
																				//lbLowerBounds = lb;															// Lower bounds on the optimization variables and general linear and nonlinear constraints.
																				//ubUpperBounds = ub;															// Upper bounds on the optimization variables and general linear and nonlinear constraints.
	mConstraints = m;															// Number of general linear and nonlinear constraints.
																				//optimizationOptions = options;												// Optimization options object containing configuration and algorithmic parameters.
	kIteration = 0;																// Current iteration.
	kIterationSinceRestart = 0;													// Iterations since restart.
	fObjectiveFunction_k = fObj;												// Objective function value at kth iteration.
	fObjectiveFunction_kPlus1.resize(1);										// Objective function value at (k + 1)th iteration.
	phiLineSearchFunction_k.resize(1);											// Line search function value.
	objectiveGradient_k = gObj;								// Objective gradient values at kth iteration.
	objectiveGradient_kPlus1.resize(nDesignVariables);							// Objective gradient values at (k + 1)th iteration.
	fConstraintFunctions_k = fCon;								// Constraint function values at kth iteration.
	jConstraintJacobian_k = gCon;				// Constraint Jacobian values at kth iteration.
	nofObjEvaluations = 0;														// Number of objective function evaluations.
	nofConEvaluations = 0;														// Number of constraint function evaluations.
	nojConEvaluations = 0;														// Number of constraint Jacobian evaluations.
	nogObjEvaluations = 0;														// Number of objective gradient evaluations.
	noMinorIterations = 0;														// Number of minor iterations.
																				// Initialize equality and inequality constraint sets.
	for (int i = 0; i < mConstraints; ++i)
	{
		if (abs(ubUpperBounds[nDesignVariables + i] - lbLowerBounds[nDesignVariables + i]) <= options.majorFeasibilityTolerance)
		{
			EqualityConstraintSet.push_back(i);									// Indices of equality constraints.
		}
		else {
			if (ubUpperBounds[nDesignVariables + i] < options.infiniteBoundSize)
			{
				InequalityConstraintSetU.push_back(i);							// Indices of upper bound inequality constraints.
			}
			if (lbLowerBounds[nDesignVariables + i] > -options.infiniteBoundSize)
			{
				InequalityConstraintSetL.push_back(i);							// Indices of lower bound inequality constraints.
			}
		}
	}
	mENoEqualityConstraints = EqualityConstraintSet.size();						// Total number of auxiliary equality constraints. 
	mINoInequalityConstraintsU = InequalityConstraintSetU.size();				// Total number of auxiliary upper bound inequality constraints.
	mINoInequalityConstraintsL = InequalityConstraintSetL.size();				// Total number of auxiliary lower bound inequality constraints.
																				//rObjectiveGradient_kPlus1.resize(nDesignVariables + mINoInequalityConstraintsU + mINoInequalityConstraintsL);	// Reformatted objective gradient values at (k + 1)th iteration.
	lagrangianGradient_k.resize(nDesignVariables);	// Lagrangian gradient values at kth iteration.
	lagrangianGradientQP.resize(nDesignVariables);				// Lagrangian gradient values at kth iteration with Quadratic Programming Lagrange multipliers.
	lagrangianGradient_kPlus1.resize(nDesignVariables);				// Lagrangian gradient values at (k + 1)th iteration.
																	//rConstraintFunctions_k.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL);				// Reformatted vector of constraints at kth iteration.
																	//rConstraintFunctions_kPlus1.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL);				// Reformatted vector of constraints at (k + 1)th iteration.
																	//jrConstraintJacobian_k.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL);				// Reformatted constraint Jacobian at kth iteration.
																	//jrConstraintJacobian_kPlus1.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL);				// Reformatted constraint Jacobian at (k + 1)th iteration.
																	//for (int i = 0; i < mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL; ++i)
																	//{
																	//	jrConstraintJacobian_k[i].resize(nDesignVariables + mINoInequalityConstraintsU + mINoInequalityConstraintsL);
																	//	jrConstraintJacobian_kPlus1[i].resize(nDesignVariables + mINoInequalityConstraintsU + mINoInequalityConstraintsL);
																	//}
																	//for (int i = 0; i < mINoInequalityConstraintsU + mINoInequalityConstraintsL; ++i)
																	//{
																	//	jrConstraintJacobian_k[mENoEqualityConstraints + i][nDesignVariables + i] = 1.0;			// Initialize Jacobian of the converted constraints with respect to the slack variables to identity matrix.
																	//	jrConstraintJacobian_kPlus1[mENoEqualityConstraints + i][nDesignVariables + i] = 1.0;			// Initialize Jacobian of the converted constraints with respect to the slack variables to identity matrix.
																	//}
	zSlackVariables_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);						// Slack variable values at kth iteration.
	zSlackVariables_kPlus1.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);					// Slack variable values at (k + 1)th iteration.
	activeConstraintSet_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);								// Indices of active constraints at kth iteration.
	inactiveConstraintSet_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);		// Indices of inactive constraints at kth iteration.
	inactiveVariableSet_k.resize(nDesignVariables);								// Indices of inactive variables at kth iteration.
	for (int i = 0; i < nDesignVariables; ++i)
	{
		inactiveVariableSet_k[i] = i;											// TODO: Currently assuming all variables inactive.
	}
	mANoActiveConstraints_k = 0;													// Total number of active constraints at kth iteration.
	mINoInactiveConstraints_k = 0;			// Total number of inactive constraints at kth iteration.
	nINoInactiveVariables_k = nDesignVariables;									// Total number of inactive variables at kth iteration.  TODO: Currently assuming all variables inactive.
	hConstraintFunctions_k.resize(mENoEqualityConstraints);						// Auxiliary vector of equality constraints at kth iteration.
	hConstraintFunctions_kPlus1.resize(mENoEqualityConstraints);						// Auxiliary vector of equality constraints at (k + 1)th iteration.
	gConstraintFunctions_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);					// Auxiliary vector of inequality constraints at kth iteration.
	gConstraintFunctions_kPlus1.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);					// Auxiliary vector of inequality constraints at (k + 1)th iteration.
	jhConstraintJacobian_k.resize(mENoEqualityConstraints);						// Auxiliary equality constraint Jacobian values at kth iteration.
	jhConstraintJacobian_kPlus1.resize(mENoEqualityConstraints);			// Auxiliary equality constraint Jacobian values at (k + 1)th iteration.
	for (int i = 0; i < mENoEqualityConstraints; ++i)
	{
		jhConstraintJacobian_k[i].resize(nDesignVariables);
		jhConstraintJacobian_kPlus1[i].resize(nDesignVariables);
	}
	jgConstraintJacobian_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);					// Auxiliary inequality constraint Jacobian values at kth iteration.
	jgConstraintJacobian_kPlus1.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);			// Auxiliary inequality constraint Jacobian values at (k + 1)th iteration.
	for (int i = 0; i < mINoInequalityConstraintsU + mINoInequalityConstraintsL; ++i)
	{
		jgConstraintJacobian_k[i].resize(nDesignVariables);
		jgConstraintJacobian_kPlus1[i].resize(nDesignVariables);
	}
	constraintViolations_k.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL);				// Vector of constraint violations at kth iteration.
	maxConstraintViolation_k.resize(1);											// Maximum constraint violation at kth iteration.
	optimalityViolations_k.resize(nDesignVariables);							// Vector of optimality violations at kth iteration.
	maxOptimalityViolation_k.resize(1);											// Maximum optimality violation at kth iteration.
	maxMinorOptimalityViolation_k.resize(1);			// Maximum minor optimality violation at kth iteration.
	lambdaLagrangeMultipliersL_k.resize(nDesignVariables + mINoInequalityConstraintsU + mINoInequalityConstraintsL);				// Lower bound inequality constraint Lagrange multiplier values at kth iteration.
	lambdaLagrangeMultipliersU_k.resize(nDesignVariables + mINoInequalityConstraintsU + mINoInequalityConstraintsL);				// Upper bound inequality constraint Lagrange multiplier values at kth iteration.
	lambdaLagrangeMultipliers_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);			// Auxiliary inequality constraint Lagrange multiplier values at kth iteration.
	lambdaLagrangeMultipliers_kPlus1.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);			// Auxiliary inequality constraint Lagrange multiplier values at (k + 1)th iteration.
	lambdaLagrangeMultipliersQP.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);			// Auxiliary inequality constraint quadratic programming Lagrange multiplier values.
	muLagrangeMultipliers_k.resize(mENoEqualityConstraints);						// Auxiliary equality constraint Lagrange multiplier values at kth iteration.
	muLagrangeMultipliers_kPlus1.resize(mENoEqualityConstraints);					// Auxiliary equality constraint Lagrange multiplier values at (k + 1)th iteration.
	muLagrangeMultipliersQP.resize(mENoEqualityConstraints);		 				// Auxiliary equality constraint quadratic programming Lagrange multiplier values.
	dxSearchDirection_k.resize(nDesignVariables);								// Search direction with respect to the design variables and slack variables.
	dzSearchDirection_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);					// Search direction with respect to the slack variables.
	dmuSearchDirection_k.resize(mENoEqualityConstraints);						// Search direction with respect to the auxiliary equality constraint Lagrange multipliers.
	dlambdaSearchDirection_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);					// Search direction with respect to the auxiliary inequality constraint Lagrange multipliers.
	phPenalties_k.resize(mENoEqualityConstraints, options.penalty);							// Penalty parameters for auxiliary equality constraints at kth iteration.
																				//phPenalties_kPlus1.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL, 1.0);							// Penalty parameters for auxiliary equality constraints at (k + 1)th iteration.
	pgPenalties_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL, options.penalty);						// Penalty parameters for auxiliary inequality constraints.
	augmentedLagrangianGradientX_k.resize(nDesignVariables);					// Augmented Lagrangian gradient with respect to design variables at kth iteration.
	augmentedLagrangianGradientXQP.resize(nDesignVariables);					// Augmented Lagrangian gradient with respect to design variables at kth iteration with Quadratic Programming Lagrange multipliers.
	augmentedLagrangianGradientX_kPlus1.resize(nDesignVariables);				// Augmented Lagrangian gradient with respect to design variables at (k + 1)th iteration.
	augmentedLagrangianGradientZ_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);		// Augmented Lagrangian gradient with respect to slack variables at kth iteration.
	augmentedLagrangianGradientZQP.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);		// Augmented Lagrangian gradient with respect to slack variables at kth iteration with Quadratic Programming Lagrange multipliers.
	augmentedLagrangianGradientZ_kPlus1.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);	// Augmented Lagrangian gradient with respect to slack variables at (k + 1)th iteration.
	augmentedLagrangianGradientMu_k.resize(mENoEqualityConstraints);			// Augmented Lagrangian gradient with respect to the auxiliary equality constraint Lagrange multipliers at kth iteration.
	augmentedLagrangianGradientMuQP.resize(mENoEqualityConstraints);			// Augmented Lagrangian gradient with respect to the auxiliary equality constraint Lagrange multipliers at kth iteration with Quadratic Programming Lagrange multipliers.
	augmentedLagrangianGradientMu_kPlus1.resize(mENoEqualityConstraints);		// Augmented Lagrangian gradient with respect to the auxiliary equality constraint Lagrange multipliers at (k + 1)th iteration.
	augmentedLagrangianGradientLambda_k.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);		// Augmented Lagrangian gradient with respect to the auxiliary inequality constraint Lagrange multipliers at kth iteration.
	augmentedLagrangianGradientLambdaQP.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);		// Augmented Lagrangian gradient with respect to the auxiliary inequality constraint Lagrange multipliers at kth iteration with Quadratic Programming Lagrange multipliers.
	augmentedLagrangianGradientLambda_kPlus1.resize(mINoInequalityConstraintsU + mINoInequalityConstraintsL);// Augmented Lagrangian gradient with respect to the auxiliary inequality constraint Lagrange multipliers at (k + 1)th iteration.
	phiPrime0_k.resize(1);														// Line search function gradient at kth iteration.
	rhoFactor = /*0.2*/ 0.0001;															// Line search factor, rho, used to determine sufficient decrease.
	betaFactor = 0.9;															// Line search factor, beta, used in curvature condition.
	etaFactor = (1.0 + sqrt(5.0)) / 2.0;										// Line search factor, eta, used to decrease step size.
	sigmaOneFactor = 0.0002;													// Search direction factor, sigmaOneFactor, used to determine if direction of descent.
	sigmaTwoFactor = sqrt(1000.0) * 1000.0;										// Search direction factor, sigmaTwoFactor, used to determine if direction of descent.
	alphaStepSize_k.resize(1, 0.0);														// Step size value at kth iteration.
	inverseHessianScaling_k.resize(1, options.scaling);												// Inverse Hessian scale factor at kth iteration.
	lineSearchProcedure_k = "N/A";				// Procedure used to calculate accepted step size at kth iteration.
	code.resize(0);						// Vector of code characters indicating what happened during the course of the major iteration.

												// Augmented Lagrangian specific variables.
	qVector.resize(nDesignVariables + mINoInequalityConstraintsU + mINoInequalityConstraintsL);											// Working vector for l-bfgs design variable and slack variable search direction calculations.
	rVector.resize(nDesignVariables + mINoInequalityConstraintsU + mINoInequalityConstraintsL);											// Working vector for l-bfgs design variable and slack variable search direction calculations.
	correctionHistory.resize(0);												// Vector used to keep track of what correction corresponds to what iteration.
	sVectors.resize(options.noCorrections);												// S correction vectors for l-bfgs design variable search direction calculations. 
	for (int i = 0; i < options.noCorrections; ++i)
	{
		sVectors[i].resize(nDesignVariables);
	}
	yVectors.resize(options.noCorrections);												// Y correction vectors for l-bfgs design variable search direction calculations. 
	for (int i = 0; i < options.noCorrections; ++i)
	{
		yVectors[i].resize(nDesignVariables);
	}
	directionMagnitude = 0.0;				// Working variable representing the search direction magnitude.
	gradientMagnitude = 0.0;				// Working variable representing the gradient magnitude.
	directionDotGradient = 0.0;				// Working variable representing dot product of search direction and gradient.
	bVector_k.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL);							// Vector used by Lagrange multiplier sub-problem.
	BMatrix_k.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL);							// Matrix used by Lagrange multiplier sub-problem.
	for (int i = 0; i < mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL; ++i)
	{
		BMatrix_k[i].resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL);
	}
	lagrangeMultiplierSearchDirection.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL);							// Vector used by Lagrange multiplier sub-problem.
	cVector_k.resize(1);							// Vector used by penalty parameter sub-problem.
	CMatrix_k.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL);							// Vector used by penalty parameter sub-problem.
	pSolution.resize(mENoEqualityConstraints + mINoInequalityConstraintsU + mINoInequalityConstraintsL + 1, 0.0);						// Penalty parameter sub-problem solution vector.

	//outputSeconds = 0.0;
	//objectiveSeconds = 0.0;
	//gradientSeconds = 0.0;
	//constraintSeconds = 0.0;
	//jacobianSeconds = 0.0;
	//lasoSeconds = 0.0;

}

// Define default destructor for ProblemAL class. 
LASO::ProblemAL::~ProblemAL()
{
	if (options.output)
	{
		outputFile.close();
		solutionFile.close();
	}
}

