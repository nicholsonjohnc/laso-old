#include <math.h>
#include <algorithm>
#include "SolverAL.h"
#include "RecursionLBFGS.h"
#include "FunctionsLM.h"
#include "FunctionsP.h"
#include <string>
#include <iostream>
#include <iomanip>

//#include <armadillo>
//#include <boost/numeric/ublas/matrix.hpp>
//#include <boost/numeric/ublas/vector.hpp>
//#include <boost/numeric/ublas/lu.hpp>
//#include <Eigen/Dense>


// Define default constructor for LBFGSB class. 
LASO::SolverAL::SolverAL()
{

}

// Main function used to solve an optimization problem.
// TODO: Unit Test.
void LASO::SolverAL::solve(ProblemAL &problem)
{
	// Set start time.
	//problem.startTime = highresolutionclock::now();

	// Apply projection operator to xDesignVariables.
	applyProjectionOperator(problem.xDesignVariables_k,
		problem.nDesignVariables, problem.lbLowerBounds, problem.ubUpperBounds);

	// Evaluate fObjectiveFunction at xDesignVariables.
	//problem.intermediateTime1 = highresolutionclock::now();
	problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
	//problem.intermediateTime2 = highresolutionclock::now();
	//problem.objectiveSeconds += (std::chrono::duration_cast<std::chrono::milliseconds>(problem.intermediateTime2 - problem.intermediateTime1).count()) / 1000.0;
	problem.nofObjEvaluations++;

	// Evaluate hConstraintFunctions and gConstraintFunctions at xDesignVariables.
	evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
		problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);

	// Calculate the maximum constraint violation, Fbar.
	calculateMaxFeasibilityViolation(problem.hConstraintFunctions_k,
		problem.gConstraintFunctions_k, problem.lambdaLagrangeMultipliers_k,
		problem.maxConstraintViolation_k, problem);

	// Evaluate reformatted objective gradient.
	evaluateObjectiveGradient(problem.xDesignVariables_k,
		problem.objectiveGradient_k, problem);

	// Evaluate auxiliary constraint Jacobians, Jh(x) and Jg(x).
	evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
		problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);

	// Evaluate zSlackVariables given gConstraintFunctions.
	// TODO: Unit Test.
	evaluateSlackVariables(problem.gConstraintFunctions_k,
		problem.lambdaLagrangeMultipliers_k, problem.pgPenalties_k,
		problem.zSlackVariables_k, problem);

	// Evaluate the gradient of the augmented Lagrangian function with respect to the design variables, slack variables, and Lagrange multipliers.
	evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
		problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
		problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
		problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
		problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
		problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
		problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
		problem);

	// Evaluate the gradient of the Lagrangian function with respect to the design variables.
	evaluateLagrangianGradient(problem.objectiveGradient_k,
		problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
		problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
		problem.lagrangianGradient_k, problem);

	// Identify active / inactive variable set.
	identifyActiveInactiveVariableSet(problem.xDesignVariables_k,
		problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
		problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k,
		problem.inactiveVariableSet_k, problem);

	// Estimate active / inactive constraint set.
	estimateActiveInactiveConstraintSet(problem.zSlackVariables_k, 
		problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
		problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
		problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
		problem);

	// Calculate the maximum optimality violation, Obar.
	calculateMaxOptimalityViolation(problem.lagrangianGradient_k,
		problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
		problem.maxOptimalityViolation_k, problem);

	// Output optimization problem status.
	outputProblemStatus(problem);

	// Check if current point is a KKT point.
	// TODO: Unit Test. 
	while (!stoppingCriteria(
		problem.maxConstraintViolation_k,
		problem.maxOptimalityViolation_k,
		problem.kIteration,
		problem))
	{
		// Evaluate bVector.
		evaluateBVector(problem.kIterationSinceRestart,
			problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
			problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
			problem.lagrangianGradient_k, problem.sVectors, problem.yVectors,
			problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
			problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
			problem.correctionHistory, problem.options.noCorrections,
			problem.bVector_k, problem);

		// Evaluate BMatrix.
		evaluateBMatrix(problem.kIterationSinceRestart,
			problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
			problem.sVectors, problem.yVectors,
			problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
			problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
			problem.correctionHistory, problem.options.noCorrections,
			problem.BMatrix_k, problem);

		// Evaluate Lagrange multiplier search direction.
		// TODO: Unit Test.
		evaluateLagrangeMultiplierSearchDirection(problem.BMatrix_k, problem.bVector_k,
			problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
			problem.dmuSearchDirection_k, problem.dlambdaSearchDirection_k, problem);

		// Function that evaluates the QP Lagrange multipliers.
		// TODO: Unit Test.
		evaluateLagrangeMultipliersQP(
			problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
			problem.dmuSearchDirection_k, problem.dlambdaSearchDirection_k,
			problem.muLagrangeMultipliersQP, problem.lambdaLagrangeMultipliersQP,
			problem);

		// Evaluate the gradient of the augmented Lagrangian function with respect to the design variables, slack variables, and Lagrange multipliers.
		evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
			problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
			problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
			problem.muLagrangeMultipliersQP, problem.lambdaLagrangeMultipliersQP,
			problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
			problem.augmentedLagrangianGradientXQP, problem.augmentedLagrangianGradientZQP,
			problem.augmentedLagrangianGradientMuQP, problem.augmentedLagrangianGradientLambdaQP,
			problem);

		// Evaluate the gradient of the Lagrangian function with respect to the design variables and slack variables.
		evaluateLagrangianGradient(problem.objectiveGradient_k,
			problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
			problem.muLagrangeMultipliersQP, problem.lambdaLagrangeMultipliersQP,
			problem.lagrangianGradientQP, problem);

		// Calculate design variable search direction.
		evaluateInactiveDesignVariableSearchDirection(problem.kIterationSinceRestart,
			problem.lagrangianGradientQP, problem.sVectors, problem.yVectors,
			problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
			problem.correctionHistory, problem.options.noCorrections,
			problem.dxSearchDirection_k, problem);

		// Calculate slack variable search direction.
		evaluateInactiveSlackVariableSearchDirection(problem.dxSearchDirection_k,
			problem.gConstraintFunctions_k, problem.jgConstraintJacobian_k,
			problem.inactiveConstraintSet_k, problem.mINoInactiveConstraints_k,
			problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
			problem.zSlackVariables_k, problem.dzSearchDirection_k, problem);

		//solveQP(problem);

		// Calculate phiPrime0, i.e. line search function gradient.
		evaluatePhiPrime0LineSearchFunctionGradient(
			problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
			problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k,
			problem.augmentedLagrangianGradientMu_k, problem.dmuSearchDirection_k,
			problem.augmentedLagrangianGradientLambda_k, problem.dlambdaSearchDirection_k,
			problem.phiPrime0_k, problem);

		// Evaluate cVector.
		evaluateCVector(problem.lagrangianGradientQP,
			problem.lagrangianGradient_k, problem.dxSearchDirection_k,
			problem.lambdaLagrangeMultipliers_k, problem.dzSearchDirection_k,
			problem.hConstraintFunctions_k, problem.dmuSearchDirection_k,
			problem.gConstraintFunctions_k, problem.zSlackVariables_k,
			problem.dlambdaSearchDirection_k, problem.cVector_k, problem);

		// Evaluate CMatrix.
		evaluateCMatrix(problem.dxSearchDirection_k, problem.dzSearchDirection_k,
			problem.jhConstraintJacobian_k, problem.hConstraintFunctions_k,
			problem.jgConstraintJacobian_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k,
			problem.CMatrix_k, problem);

		// Evaluate penalty parameters.
		// TODO: Unit Test
		updatePenaltyParameterVectors(problem.CMatrix_k, problem.cVector_k, problem.phiPrime0_k,
			problem.lagrangianGradientQP, problem.dxSearchDirection_k,
			problem.phPenalties_k, problem.pgPenalties_k, problem);

		// Evaluate the gradient of the augmented Lagrangian function with respect to the design variables, slack variables, and Lagrange multipliers.
		evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
			problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
			problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
			problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
			problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
			problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
			problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
			problem);

		// Calculate phiPrime0, i.e. line search function gradient.
		evaluatePhiPrime0LineSearchFunctionGradient(
			problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
			problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k,
			problem.augmentedLagrangianGradientMu_k, problem.dmuSearchDirection_k,
			problem.augmentedLagrangianGradientLambda_k, problem.dlambdaSearchDirection_k,
			problem.phiPrime0_k, problem);

		// Ensure calculated search direction is that of descent.
		//ensureInactiveSearchDirectionThatOfDescent(problem.dxSearchDirection_k,
		//	problem.augmentedLagrangianGradientX_k, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
		//	problem.correctionHistory, problem);

		// Initialize step size.
		initializeStepSize(problem.alphaStepSize_k,
			problem.zSlackVariables_k, problem.dzSearchDirection_k,
			problem.lambdaLagrangeMultipliers_k, problem.dlambdaSearchDirection_k,
			problem);

		// Identify step size.
		// TODO: Unit Test.
		identifyStepSize(problem.alphaStepSize_k, problem.kIterationSinceRestart,
			problem.xDesignVariables_k, problem.zSlackVariables_k,
			problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
			problem.dxSearchDirection_k, problem.dzSearchDirection_k,
			problem.dmuSearchDirection_k, problem.dlambdaSearchDirection_k,
			problem.phPenalties_k, problem.pgPenalties_k,
			problem.fObjectiveFunction_k,
			problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
			problem.phiPrime0_k,
			problem.xDesignVariables_kPlus1, problem.zSlackVariables_kPlus1,
			problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
			problem.fObjectiveFunction_kPlus1,
			problem.hConstraintFunctions_kPlus1, problem.gConstraintFunctions_kPlus1,
			problem);

		// Evaluate objective gradient.
		evaluateObjectiveGradient(&problem.xDesignVariables_kPlus1[0],
			&problem.objectiveGradient_kPlus1[0], problem);

		// Evaluate auxiliary constraint Jacobians, Jh(x) and Jg(x).
		evaluateAuxiliaryJacobians(&problem.xDesignVariables_kPlus1[0],
			problem.jhConstraintJacobian_kPlus1, problem.jgConstraintJacobian_kPlus1, problem);

		// Evaluate zSlackVariables given gConstraintFunctions.
		// TODO: Unit Test.
		//evaluateSlackVariables(problem.gConstraintFunctions_kPlus1,
		//	problem.lambdaLagrangeMultipliers_kPlus1, problem.pgPenalties_k,
		//	problem.zSlackVariables_kPlus1, problem);

		// Evaluate the gradient of the augmented Lagrangian function with respect to the design variables, slack variables, and Lagrange multipliers.
		//evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
		//	problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
		//	problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
		//	problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
		//	problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_kPlus1,
		//	problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
		//	problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
		//	problem);

		// Evaluate the gradient of the Lagrangian function with respect to the design variables and slack variables.
		evaluateLagrangianGradient(problem.objectiveGradient_k,
			problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
			problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
			problem.lagrangianGradient_k, problem);

		// Evaluate the gradient of the augmented Lagrangian function with respect to the design variables, slack variables, and Lagrange multipliers.
		evaluateAugmentedLagrangianGradients(&problem.objectiveGradient_kPlus1[0],
			problem.hConstraintFunctions_kPlus1, problem.gConstraintFunctions_kPlus1,
			problem.jhConstraintJacobian_kPlus1, problem.jgConstraintJacobian_kPlus1,
			problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
			problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_kPlus1,
			problem.augmentedLagrangianGradientX_kPlus1, problem.augmentedLagrangianGradientZ_kPlus1,
			problem.augmentedLagrangianGradientMu_kPlus1, problem.augmentedLagrangianGradientLambda_kPlus1,
			problem);

		// Evaluate the gradient of the Lagrangian function with respect to the design variables and slack variables.
		evaluateLagrangianGradient(&problem.objectiveGradient_kPlus1[0],
			problem.jhConstraintJacobian_kPlus1, problem.jgConstraintJacobian_kPlus1,
			problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
			problem.lagrangianGradient_kPlus1, problem);

		// Update correction vectors.
		// TODO: Unit Test.
		updateCorrectionVectors(problem.xDesignVariables_k, problem.xDesignVariables_kPlus1,
			problem.lagrangianGradient_k, problem.lagrangianGradient_kPlus1,
			problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
			problem.correctionHistory, problem.options.noCorrections,
			problem.sVectors, problem.yVectors,
			problem);

		// Set variables for next iteration.
		setVariablesForNextIteration(
			problem.fObjectiveFunction_k, problem.fObjectiveFunction_kPlus1,
			problem.xDesignVariables_k, problem.xDesignVariables_kPlus1,
			problem.zSlackVariables_k, problem.zSlackVariables_kPlus1,
			problem.muLagrangeMultipliers_k, problem.muLagrangeMultipliers_kPlus1,
			problem.lambdaLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_kPlus1,
			problem.hConstraintFunctions_k, problem.hConstraintFunctions_kPlus1,
			problem.gConstraintFunctions_k, problem.gConstraintFunctions_kPlus1,
			problem.objectiveGradient_k, problem.objectiveGradient_kPlus1,
			problem.jhConstraintJacobian_k, problem.jhConstraintJacobian_kPlus1,
			problem.jgConstraintJacobian_k, problem.jgConstraintJacobian_kPlus1,
			problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientX_kPlus1,
			problem.augmentedLagrangianGradientZ_k, problem.augmentedLagrangianGradientZ_kPlus1,
			problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientMu_kPlus1,
			problem.augmentedLagrangianGradientLambda_k, problem.augmentedLagrangianGradientLambda_kPlus1,
			problem.lagrangianGradient_k, problem.lagrangianGradient_kPlus1,
			problem);

		// Increment kIteration and kIterationSinceRestart.
		++problem.kIteration;
		++problem.kIterationSinceRestart;

		// Calculate the maximum constraint violation, Fbar.
		calculateMaxFeasibilityViolation(problem.hConstraintFunctions_k,
			problem.gConstraintFunctions_k, problem.lambdaLagrangeMultipliers_k,
			problem.maxConstraintViolation_k, problem);

		// Identify active / inactive variable set.
		identifyActiveInactiveVariableSet(problem.xDesignVariables_k,
			problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
			problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k,
			problem.inactiveVariableSet_k, problem);

		// Estimate active / inactive constraint set.
		estimateActiveInactiveConstraintSet(problem.zSlackVariables_k,
			problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
			problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
			problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
			problem);

		// Calculate the maximum optimality violation, Obar.
		calculateMaxOptimalityViolation(problem.lagrangianGradient_k,
			problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
			problem.maxOptimalityViolation_k, problem);

		// Output optimization problem status.
		outputProblemStatus(problem);

	}

	// Set finish time.
	//problem.finishTime = highresolutionclock::now();

	// Output optimization problem solution.
	outputProblemSolution(problem);

}

// Function used to solve QP sub-problem.
void LASO::SolverAL::solveQP(ProblemAL &problem)
{
	boolean stillRemovingConstraints = true;
	boolean stillAddingConstraints = true;
	integer kIteration = 0;

	do
	{
		do
		{
			// Evaluate bVector.
			evaluateBVector(problem.kIterationSinceRestart,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.lagrangianGradient_k, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.bVector_k, problem);

			// Evaluate BMatrix.
			evaluateBMatrix(problem.kIterationSinceRestart,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.BMatrix_k, problem);

			// Evaluate Lagrange multiplier search direction.
			// TODO: Unit Test.
			evaluateLagrangeMultiplierSearchDirection(problem.BMatrix_k, problem.bVector_k,
				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
				problem.dmuSearchDirection_k, problem.dlambdaSearchDirection_k, problem);

			// Function that evaluates the QP Lagrange multipliers.
			// TODO: Unit Test.
			evaluateLagrangeMultipliersQP(
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.dmuSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.muLagrangeMultipliersQP, problem.lambdaLagrangeMultipliersQP,
				problem);

			stillRemovingConstraints = removeConstraint(problem);

			kIteration++;

		} while (stillRemovingConstraints);

		// Evaluate the gradient of the augmented Lagrangian function with respect to the design variables, slack variables, and Lagrange multipliers.
		evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
			problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
			problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
			problem.muLagrangeMultipliersQP, problem.lambdaLagrangeMultipliersQP,
			problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
			problem.augmentedLagrangianGradientXQP, problem.augmentedLagrangianGradientZQP,
			problem.augmentedLagrangianGradientMuQP, problem.augmentedLagrangianGradientLambdaQP,
			problem);

		// Evaluate the gradient of the Lagrangian function with respect to the design variables and slack variables.
		evaluateLagrangianGradient(problem.objectiveGradient_k,
			problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
			problem.muLagrangeMultipliersQP, problem.lambdaLagrangeMultipliersQP,
			problem.lagrangianGradientQP, problem);

		// Calculate design variable search direction.
		evaluateInactiveDesignVariableSearchDirection(problem.kIterationSinceRestart,
			problem.lagrangianGradientQP, problem.sVectors, problem.yVectors,
			problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
			problem.correctionHistory, problem.options.noCorrections,
			problem.dxSearchDirection_k, problem);

		// Calculate slack variable search direction.
		evaluateInactiveSlackVariableSearchDirection(problem.dxSearchDirection_k,
			problem.gConstraintFunctions_k, problem.jgConstraintJacobian_k,
			problem.inactiveConstraintSet_k, problem.mINoInactiveConstraints_k,
			problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
			problem.zSlackVariables_k, problem.dzSearchDirection_k, problem);

		//if (removeConstraint(problem))
		//{
		//	solveAgain = true;
		//}
		//else if (addConstraint(problem))
		//{
		//	solveAgain = true;
		//}

		stillAddingConstraints = addConstraint(problem);

		kIteration++;

	} while (stillAddingConstraints);

	problem.noMinorIterations = kIteration;
}

bool LASO::SolverAL::removeConstraint(ProblemAL &problem)
{
	doublereal smallestLagrangeMultiplier = 0.0;
	integer smallestLagrangeMultiplierIndex = 0;
	integer smallestLagrangeMultiplierConstraint = 0;

	// Iterate over active constraint set.
	for (integer i = 0; i < problem.mANoActiveConstraints_k; ++i)
	{
		// If QP Lagrange multipler is smaller than smallest Lagrange multiplier, update smallest.
		if (problem.lambdaLagrangeMultipliersQP[problem.activeConstraintSet_k[i]] < smallestLagrangeMultiplier)
		{
			smallestLagrangeMultiplier = problem.lambdaLagrangeMultipliersQP[problem.activeConstraintSet_k[i]];
			smallestLagrangeMultiplierIndex = i;
			smallestLagrangeMultiplierConstraint = problem.activeConstraintSet_k[i];
		}
	}

	// If smallest Lagrange multiplier less than 0, remove corresponding constraint from active constraint set and add it to inactive constraint set.
	if (smallestLagrangeMultiplier < 0.0)
	{
		problem.activeConstraintSet_k.erase(problem.activeConstraintSet_k.begin() + smallestLagrangeMultiplierIndex);
		problem.mANoActiveConstraints_k--;

		problem.inactiveConstraintSet_k.push_back(smallestLagrangeMultiplierConstraint);
		problem.mINoInactiveConstraints_k++;
		problem.lambdaLagrangeMultipliersL_k[problem.nDesignVariables + smallestLagrangeMultiplierConstraint] = 0.0;
		problem.lambdaLagrangeMultipliersU_k[problem.nDesignVariables + smallestLagrangeMultiplierConstraint] = 0.0;
		problem.lambdaLagrangeMultipliers_k[smallestLagrangeMultiplierConstraint] = 0.0;
		problem.dlambdaSearchDirection_k[smallestLagrangeMultiplierConstraint] = 0.0;

		std::sort(problem.inactiveConstraintSet_k.begin(), problem.inactiveConstraintSet_k.end());

		return true;
	}
	else
	{
		return false;
	}
}

bool LASO::SolverAL::addConstraint(ProblemAL &problem)
{
	doublereal smallestStep = 1.0;
	doublereal computedStep = 0.0;
	integer smallestStepIndex = 0;
	integer smallestStepConstraint = 0;
	doublereal jgdotdx = 0.0;
	doublereal jgdotx = 0.0;

	// Iterate over inactive constraint set.
	for (integer i = 0; i < problem.mINoInactiveConstraints_k; ++i)
	{
		// Compute jgdotdx
		jgdotdx = 0.0;
		for (integer j = 0; j < problem.nINoInactiveVariables_k; ++j)
		{
			jgdotdx += problem.jgConstraintJacobian_k[problem.inactiveConstraintSet_k[i]][problem.inactiveVariableSet_k[j]] * problem.dxSearchDirection_k[problem.inactiveVariableSet_k[j]];
		}

		// If jgdotdx > 0, proceed.
		if (jgdotdx > 0.0)
		{
			// Compute jgdotx
			jgdotx = 0.0;
			for (integer j = 0; j < problem.nINoInactiveVariables_k; ++j)
			{
				jgdotx += problem.jgConstraintJacobian_k[problem.inactiveConstraintSet_k[i]][problem.inactiveVariableSet_k[j]] * problem.xDesignVariables_k[problem.inactiveVariableSet_k[j]];
			}

			// Compute step.
			computedStep = (-problem.gConstraintFunctions_k[problem.inactiveConstraintSet_k[i]] - jgdotx) / jgdotdx;

			// If computed step is smaller than smallest step, update smallest.
			if (computedStep < smallestStep)
			{
				smallestStep = computedStep;
				smallestStepIndex = i;
				smallestStepConstraint = problem.inactiveConstraintSet_k[i];
			}
		}
	}

	// If smallest step less than 1, add corresponding constraint to active constraint set from inactive constraint set.
	if (smallestStep < 1.0)
	{
		problem.inactiveConstraintSet_k.erase(problem.inactiveConstraintSet_k.begin() + smallestStepIndex);
		problem.mINoInactiveConstraints_k--;

		problem.activeConstraintSet_k.push_back(smallestStepConstraint);
		problem.mANoActiveConstraints_k++;
		problem.lambdaLagrangeMultipliersL_k[problem.nDesignVariables + smallestStepConstraint] = problem.augmentedLagrangianGradientZ_k[smallestStepConstraint];
		problem.lambdaLagrangeMultipliersU_k[problem.nDesignVariables + smallestStepConstraint] = 0.0;
		problem.dzSearchDirection_k[smallestStepConstraint] = std::max(-problem.augmentedLagrangianGradientZ_k[smallestStepConstraint], -problem.zSlackVariables_k[smallestStepConstraint]);

		std::sort(problem.activeConstraintSet_k.begin(), problem.activeConstraintSet_k.end());

		return true;
	}
	else
	{
		return false;
	}
}

bool LASO::SolverAL::stoppingCriteriaQP(ProblemAL &problem)
{
	return true;
}

// Function that ensures all variables are on or within their bounds.
void LASO::SolverAL::applyProjectionOperator(double *variables,
	integer noVariables, vector &lowerBounds, vector &upperBounds)
{
	doublereal vi = 0.0;
	doublereal zi = 0.0;

	for (integer i = 0; i < noVariables; ++i)
	{
		vi = variables[i];

		if (vi <= lowerBounds[i])
		{
			variables[i] = lowerBounds[i];
		}
		else if (vi >= upperBounds[i])
		{
			variables[i] = upperBounds[i];
		}
		else
		{
			variables[i] = vi;
		}
	}
}

// Function that ensures all variables are on or within constant bounds.
void LASO::SolverAL::applyProjectionOperatorConstantBounds(vector &variables,
	integer noVariables, doublereal lowerBound, doublereal upperBound)
{
	doublereal vi = 0.0;
	doublereal zi = 0.0;

	for (integer i = 0; i < noVariables; ++i)
	{
		vi = variables[i];

		if (vi <= lowerBound)
		{
			variables[i] = lowerBound;
		}
		else if (vi >= upperBound)
		{
			variables[i] = upperBound;
		}
		else
		{
			variables[i] = vi;
		}
	}
}

// Function that evaluates the auxiliary vectors of constraints, h(x) and g(x).
void LASO::SolverAL::evaluateAuxiliaryConstraints(double *xDesignVariables,
	vector &hConstraintFunctions, vector &gConstraintFunctions, ProblemAL &problem)
{

	// Evaluate fConstraintFunctions at xDesignVariables and increment nofConEvaluations.
	//problem.intermediateTime1 = highresolutionclock::now();
	problem.functions.conFun(xDesignVariables, problem.fConstraintFunctions_k);
	//problem.intermediateTime2 = highresolutionclock::now();
	//problem.constraintSeconds += (std::chrono::duration_cast<std::chrono::milliseconds>(problem.intermediateTime2 - problem.intermediateTime1).count()) / 1000.0;
	problem.nofConEvaluations++;

	// Form hConstraintFunctions from fConstraintFunctions.
	integer constraintNo = 0;
	integer limit = problem.mENoEqualityConstraints;
	for (integer i = 0; i < limit; ++i)
	{
		constraintNo = problem.EqualityConstraintSet[i];
		hConstraintFunctions[i] = problem.fConstraintFunctions_k[constraintNo]
			- 0.5 * (problem.lbLowerBounds[problem.nDesignVariables + constraintNo] + problem.ubUpperBounds[problem.nDesignVariables + constraintNo]);
	}
	// Form gConstraintFunctions from fConstraintFunctions.
	constraintNo = 0;
	limit = problem.mINoInequalityConstraintsU;
	for (integer i = 0; i < limit; ++i)
	{
		constraintNo = problem.InequalityConstraintSetU[i];
		gConstraintFunctions[i] = problem.fConstraintFunctions_k[constraintNo] - problem.ubUpperBounds[problem.nDesignVariables + constraintNo];
	}
	constraintNo = 0;
	limit = problem.mINoInequalityConstraintsL;
	for (integer i = 0; i < limit; ++i)
	{
		constraintNo = problem.InequalityConstraintSetL[i];
		gConstraintFunctions[problem.mINoInequalityConstraintsU + i] = problem.lbLowerBounds[problem.nDesignVariables + constraintNo] - problem.fConstraintFunctions_k[constraintNo];
	}

}

// Function that evaluates zSlackVariables given gConstraintFunctions.
void LASO::SolverAL::evaluateSlackVariables(vector &gConstraintFunctions,
	vector &lambdaLagrangeMultipliers, vector &pgPenalties,
	vector &zSlackVariables, ProblemAL &problem)
{
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		if (pgPenalties[i] == 0.0)
		{
			zSlackVariables[i] = std::max(0.0, -gConstraintFunctions[i]);
		}
		else
		{
			zSlackVariables[i] = std::max(0.0,
				-gConstraintFunctions[i] - lambdaLagrangeMultipliers[i] / pgPenalties[i]);
		}
	}
}

// Function that evaluates the gradient of the objective function with respect to the design variables.
void LASO::SolverAL::evaluateObjectiveGradient(double *xDesignVariables,
	double *objectiveGradient, ProblemAL &problem)
{
	// Evaluate objectiveGradient at xDesignVariables.
	//problem.intermediateTime1 = highresolutionclock::now();
	problem.functions.objGrad(xDesignVariables, objectiveGradient);
	//problem.intermediateTime2 = highresolutionclock::now();
	//problem.gradientSeconds += (std::chrono::duration_cast<std::chrono::milliseconds>(problem.intermediateTime2 - problem.intermediateTime1).count()) / 1000.0;
	problem.nogObjEvaluations++;
}

// Function that evaluates the auxiliary constraint Jacobians, Jh(x) and Jg(x).
void LASO::SolverAL::evaluateAuxiliaryJacobians(double *xDesignVariables,
	matrix &jhConstraintJacobian, matrix &jgConstraintJacobian, ProblemAL &problem)
{

	// Evaluate jConstraintJacobian at xDesignVariables and increment nojConEvaluations.
	//problem.intermediateTime1 = highresolutionclock::now();
	problem.functions.conJac(xDesignVariables, problem.jConstraintJacobian_k);
	//problem.intermediateTime2 = highresolutionclock::now();
	//problem.jacobianSeconds += (std::chrono::duration_cast<std::chrono::milliseconds>(problem.intermediateTime2 - problem.intermediateTime1).count()) / 1000.0;
	problem.nojConEvaluations++;

	// Form jhConstraintJacobian from jConstraintJacobian.
	integer constraintNo = 0;
	integer limit = problem.mENoEqualityConstraints;
	for (integer i = 0; i < limit; ++i)
	{
		constraintNo = problem.EqualityConstraintSet[i];
		for (integer j = 0; j < problem.nDesignVariables; ++j)
		{
			jhConstraintJacobian[i][j] = problem.jConstraintJacobian_k[j * problem.mConstraints + constraintNo];
		}
	}
	// Form jgConstraintJacobian from jrConstraintJacobian.
	constraintNo = 0;
	limit = problem.mINoInequalityConstraintsU;
	for (integer i = 0; i < limit; ++i)
	{
		constraintNo = problem.InequalityConstraintSetU[i];
		for (integer j = 0; j < problem.nDesignVariables; ++j)
		{
			jgConstraintJacobian[i][j] = problem.jConstraintJacobian_k[j * problem.mConstraints + constraintNo];
		}
	}
	constraintNo = 0;
	limit = problem.mINoInequalityConstraintsL;
	for (integer i = 0; i < limit; ++i)
	{
		constraintNo = problem.InequalityConstraintSetL[i];
		for (integer j = 0; j < problem.nDesignVariables; ++j)
		{
			jgConstraintJacobian[problem.mINoInequalityConstraintsU + i][j] = -problem.jConstraintJacobian_k[j * problem.mConstraints + constraintNo];
		}
	}

}

// Function that calculates the maximum constraint violation, Fbar.
void LASO::SolverAL::calculateMaxFeasibilityViolation(vector &hConstraintFunctions,
	vector &gConstraintFunctions, vector &lambdaLagrangeMultipliers,
	vector &maxConstraintViolation, ProblemAL &problem)
{
	// Form constraintViolations from hConstraintFunctions.
	integer iStart = 0;
	integer iEnd = problem.mENoEqualityConstraints;
	for (integer i = iStart; i < iEnd; ++i)
	{
		problem.constraintViolations_k[i] = abs(hConstraintFunctions[i]);
	}
	// Form constraintViolations from gConstraintFunctions and lambdaLagrangeMultipliers.
	iStart = 0;
	iEnd = problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL;
	for (integer i = iStart; i < iEnd; ++i)
	{
		problem.constraintViolations_k[problem.mENoEqualityConstraints + i] = abs(std::max(gConstraintFunctions[i], -lambdaLagrangeMultipliers[i]));
	}
	// Form maxConstraintViolation from constraintViolations if constrained problem.
	if (problem.mENoEqualityConstraints + problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL > 0)
	{
		maxConstraintViolation[0] = *std::max_element(problem.constraintViolations_k.begin(), problem.constraintViolations_k.end());
	
		// Calculate design variable magnitude.
		doublereal xMag = 0.0;
		integer iStart = 0;
		integer iEnd = problem.nDesignVariables;
		for (integer i = iStart; i < iEnd; ++i)
		{
			xMag += pow(problem.xDesignVariables_k[i], 2.0);
		}
		xMag = sqrt(xMag);

		// Normalize maxConstraintViolation.
		maxConstraintViolation[0] /= (1.0 + xMag);
	}
}

// Function that evaluates the gradient of the Lagrangian function with respect to the design variables.
void LASO::SolverAL::evaluateLagrangianGradient(double *objectiveGradient,
	matrix &jhConstraintJacobian, matrix &jgConstraintJacobian,
	vector &muLagrangeMultipliers, vector &lambdaLagrangeMultipliers,
	vector &lagrangianGradient, ProblemAL &problem)
{

	// Form lagrangianGradient from objectiveGradient, jhConstraintJacobian, 
	// jgConstraintJacobian, muLagrangeMultipliers, and lambdaLagrangeMultipliers.
	for (integer i = 0; i < problem.nDesignVariables; ++i)
	{
		lagrangianGradient[i] = objectiveGradient[i];

		for (integer j = 0; j < problem.mENoEqualityConstraints; ++j)
		{
			lagrangianGradient[i] += muLagrangeMultipliers[j] * jhConstraintJacobian[j][i];
		}

		for (integer j = 0; j < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++j)
		{
			lagrangianGradient[i] += lambdaLagrangeMultipliers[j] * jgConstraintJacobian[j][i];
		}
	}

}

// Function that calculates the maximum optimality violation, Obar.
void LASO::SolverAL::calculateMaxOptimalityViolation(vector &lagrangianGradient,
	vectorinteger &inactiveVariableSet, integer noInactiveVariables,
	vector &maxOptimalityViolation, ProblemAL &problem)
{
	for (integer i = 0; i < problem.nDesignVariables; ++i)
	{
		problem.optimalityViolations_k[i] = 0.0;
	}

	//integer variable = 0;
	for (integer i = 0; i < noInactiveVariables; ++i)
	{
		//variable = inactiveVariableSet[i];
		//if (lagrangianGradient[variable] >= 0.0)
		//{
		//	problem.optimalityViolations_k[variable] = lagrangianGradient[variable] *
		//		std::min(problem.xDesignVariables_k[variable] - problem.lbLowerBounds[variable], 1.0);
		//}
		//else
		//{
		//	problem.optimalityViolations_k[variable] = -lagrangianGradient[variable] *
		//		std::min(problem.ubUpperBounds[variable] - problem.xDesignVariables_k[variable], 1.0);
		//}
		problem.optimalityViolations_k[inactiveVariableSet[i]] = abs(lagrangianGradient[inactiveVariableSet[i]]);
	}

	// Form maxOptimalityViolation from optimalityViolations.
	maxOptimalityViolation[0] = *std::max_element(problem.optimalityViolations_k.begin(), problem.optimalityViolations_k.end());

	//// Normalization method 1.
	//if (problem.mENoEqualityConstraints + problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL > 0)
	//{
		//// Calculate lagrange multiplier magnitude.
		//doublereal lagrangeMultiplierMag = 0.0;
		//integer iStart = 0;
		//integer iEnd = problem.mENoEqualityConstraints;
		//for (integer i = iStart; i < iEnd; ++i)
		//{
		//	lagrangeMultiplierMag += pow(problem.muLagrangeMultipliers_k[i], 2.0);
		//}
		//iStart = 0;
		//iEnd = problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL;
		//for (integer i = iStart; i < iEnd; ++i)
		//{
		//	lagrangeMultiplierMag += pow(problem.lambdaLagrangeMultipliers_k[i], 2.0);
		//}
		//lagrangeMultiplierMag = sqrt(lagrangeMultiplierMag);
		//// Normalize maxOptimalityViolation.
		//maxOptimalityViolation[0] /= std::max(lagrangeMultiplierMag, 1.0);
	//}

	//// Normalization method 2 (preferred).
	//// Calculate gradient magnitude.
	//doublereal gradientMag = 0.0;
	//integer iStart = 0;
	//integer iEnd = problem.nDesignVariables;
	//for (integer i = iStart; i < iEnd; ++i)
	//{
	//	gradientMag += pow(problem.objectiveGradient_k[i], 2.0);
	//}
	//gradientMag = sqrt(gradientMag);
	//// Normalize maxOptimalityViolation.
	//maxOptimalityViolation[0] /= (1.0 + std::max(1.0 + abs(problem.fObjectiveFunction_k[0]), gradientMag));
}

// Function that returns true if current point is feasible, false otherwise.
boolean LASO::SolverAL::testFeasible(vector &maxConstraintViolation, ProblemAL &problem)
{

	if (maxConstraintViolation[0] <= problem.options.majorFeasibilityTolerance)
	{
		return true;
	}
	else
	{
		return false;
	}

}

// Function that returns true if current point is optimal, false otherwise.
boolean LASO::SolverAL::testOptimal(vector &maxOptimalityViolation, ProblemAL &problem)
{

	if (maxOptimalityViolation[0] <= problem.options.majorOptimalityTolerance)
	{
		return true;
	}
	else
	{
		return false;
	}

}

// Function that returns true if maximum number of iterations is exceeded, false otherwise.
boolean LASO::SolverAL::testIterations(integer kIteration, ProblemAL &problem)
{

	if (kIteration >= problem.options.majorIterationsLimit)
	{
		return true;
	}
	else
	{
		return false;
	}

}

// Function that returns true if stopping criteria satisfied, false otherwise.
boolean LASO::SolverAL::stoppingCriteria(vector &maxConstraintViolation, vector &maxOptimalityViolation,
	integer kIteration, ProblemAL &problem)
{
	if (testFeasible(maxConstraintViolation, problem) && testOptimal(maxOptimalityViolation, problem))
	{
		return true;
	}
	else if (testIterations(kIteration, problem))
	{
		return true;
	}
	else
	{
		return false;
	}
}

// Evaluate the gradient of the augmented Lagrangian function with respect to the design variables, slack variables, and Lagrange multipliers.
void LASO::SolverAL::evaluateAugmentedLagrangianGradients(double *objectiveGradient,
	vector &hConstraintFunctions, vector &gConstraintFunctions,
	matrix &jhConstraintJacobian, matrix &jgConstraintJacobian,
	vector &muLagrangeMultipliers, vector &lambdaLagrangeMultipliers,
	vector &phPenalties, vector &pgPenalties, vector &zSlackVariables,
	vector &augmentedLagrangianGradientX, vector &augmentedLagrangianGradientZ,
	vector &augmentedLagrangianGradientMu, vector &augmentedLagrangianGradientLambda,
	ProblemAL &problem)
{
	// Form augmentedLagrangianGradientX.
	for (integer i = 0; i < problem.nDesignVariables; ++i)
	{
		augmentedLagrangianGradientX[i] = objectiveGradient[i];

		// Add penalties for each equality constraint.
		for (integer j = 0; j < problem.mENoEqualityConstraints; ++j)
		{
			augmentedLagrangianGradientX[i] += muLagrangeMultipliers[j] * jhConstraintJacobian[j][i] + phPenalties[j] * hConstraintFunctions[j] * jhConstraintJacobian[j][i];
		}

		// Add penalties for each inequality constraint.
		for (integer j = 0; j < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++j)
		{
			augmentedLagrangianGradientX[i] += lambdaLagrangeMultipliers[j] * jgConstraintJacobian[j][i] + pgPenalties[j] * (gConstraintFunctions[j] + zSlackVariables[j]) * jgConstraintJacobian[j][i];
		}
	}

	// Form augmentedLagrangianGradientZ.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		augmentedLagrangianGradientZ[i] = lambdaLagrangeMultipliers[i] +
			pgPenalties[i] * (gConstraintFunctions[i] + zSlackVariables[i]);
	}

	// Form augmentedLagrangianGradientMu.
	for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
	{
		augmentedLagrangianGradientMu[i] = hConstraintFunctions[i];
	}

	// Form augmentedLagrangianGradientLambda.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		augmentedLagrangianGradientLambda[i] = gConstraintFunctions[i] + zSlackVariables[i];
	}
}

// Function that identifies active / inactive variable set.
void LASO::SolverAL::identifyActiveInactiveVariableSet(double *xDesignVariables,
	vector &gradientX, vector &dxSearchDirection, 
	vector &lambdaLagrangeMultipliersL, vector &lambdaLagrangeMultipliersU,
	vectorinteger &inactiveVariableSet,	ProblemAL &problem)
{
	problem.nINoInactiveVariables_k = 0;

	// Iterate over design variables.
	for (int i = 0; i < problem.nDesignVariables; ++i)
	{
		// If lower bound is active.
		if (problem.lbLowerBounds[i] <= xDesignVariables[i] &&
			xDesignVariables[i] <= problem.lbLowerBounds[i] + problem.options.majorFeasibilityTolerance &&
			gradientX[i] > 0.0)
		{
			lambdaLagrangeMultipliersL[i] = gradientX[i];
			lambdaLagrangeMultipliersU[i] = 0.0;
			dxSearchDirection[i] = std::max(-gradientX[i], problem.lbLowerBounds[i] - xDesignVariables[i]);
		}
		// If upper bound is active.
		else if (problem.ubUpperBounds[i] - problem.options.majorFeasibilityTolerance <= xDesignVariables[i] &&
			xDesignVariables[i] <= problem.ubUpperBounds[i] &&
			gradientX[i] < 0.0)
		{
			lambdaLagrangeMultipliersL[i] = 0.0;
			lambdaLagrangeMultipliersU[i] = -gradientX[i];
			dxSearchDirection[i] = std::min(-gradientX[i], problem.ubUpperBounds[i] - xDesignVariables[i]);
		}
		// If neither bound is active.
		else
		{
			inactiveVariableSet[problem.nINoInactiveVariables_k] = i;
			problem.nINoInactiveVariables_k++;
			lambdaLagrangeMultipliersL[i] = 0.0;
			lambdaLagrangeMultipliersU[i] = 0.0;
		}
	}
}

// Function that estimates active / inactive constraint set.
void LASO::SolverAL::estimateActiveInactiveConstraintSet(vector &zSlackVariables, 
	vector &gradientZ, vector &dzSearchDirection, vector &dlambdaSearchDirection,
	vector &lambdaLagrangeMultipliersL, vector &lambdaLagrangeMultipliersU, vector &lambdaLagrangeMultipliers,
	vectorinteger &activeConstraintSet, vectorinteger &inactiveConstraintSet,
	ProblemAL &problem)
{
	problem.mANoActiveConstraints_k = 0;
	problem.mINoInactiveConstraints_k = 0;
	activeConstraintSet.clear();
	inactiveConstraintSet.clear();

	// Iterate over slack variables.
	for (int i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		// If lower bound is active.
		if (0.0 <= zSlackVariables[i] &&
			zSlackVariables[i] <= problem.options.majorFeasibilityTolerance &&
			gradientZ[i] > 0.0)
		{
			activeConstraintSet.push_back(i);
			problem.mANoActiveConstraints_k++;
			lambdaLagrangeMultipliersL[problem.nDesignVariables + i] = gradientZ[i];
			lambdaLagrangeMultipliersU[problem.nDesignVariables + i] = 0.0;
			dzSearchDirection[i] = std::max(-gradientZ[i], -zSlackVariables[i]);
		}
		// If neither bound is active.
		else
		{
			inactiveConstraintSet.push_back(i);
			problem.mINoInactiveConstraints_k++;
			lambdaLagrangeMultipliersL[problem.nDesignVariables + i] = 0.0;
			lambdaLagrangeMultipliersU[problem.nDesignVariables + i] = 0.0;
			lambdaLagrangeMultipliers[i] = 0.0;
			dlambdaSearchDirection[i] = 0.0;
		}
	}
}

// Function that outputs optimization problem status.
void LASO::SolverAL::outputProblemStatus(ProblemAL &problem)
{
	// If output set to true.
	if (problem.options.output)
	{
		// Set intermediate time 1.
		//problem.intermediateTime1 = highresolutionclock::now();

		// Compute max penalty.
		doublereal phMax = 0.0;
		doublereal pgMax = 0.0;
		doublereal pMax = 0.0;
		if (problem.mENoEqualityConstraints > 0)
		{
			phMax = *std::max_element(problem.phPenalties_k.begin(), problem.phPenalties_k.end());
		}
		if (problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL > 0)
		{
			pgMax = *std::max_element(problem.pgPenalties_k.begin(), problem.pgPenalties_k.end());
		}
		pMax = std::max(phMax, pgMax);

		// Compute dHd.
		doublereal dHd = 0.0;
		for (integer i = 0; i < problem.nDesignVariables; ++i)
		{
			dHd += problem.lagrangianGradientQP[i] * problem.dxSearchDirection_k[i];
		}

		// Compute dInfNorm.
		//doublereal dMax = *std::max_element(problem.dxSearchDirection_k.begin(), problem.dxSearchDirection_k.end());
		//doublereal dMin = *std::min_element(problem.dxSearchDirection_k.begin(), problem.dxSearchDirection_k.end());
		//doublereal dInfNorm = std::max(abs(dMin), abs(dMax));

		// Compute dMag.
		doublereal dMag = 0.0;
		for (integer i = 0; i < problem.nINoInactiveVariables_k; ++i)
		{
			dMag += pow(problem.dxSearchDirection_k[problem.inactiveVariableSet_k[i]], 2.0);
		}
		dMag = sqrt(dMag);

		// Output headers.
		if (problem.kIteration < 1)
		{
			std::cout <<
				std::left << std::setw(5) << "Maj" << "\t" <<
				std::left << std::setw(5) << "Min " << "\t" <<
				std::left << std::setw(8) << "Conv" << "\t" <<
				std::left << std::setw(5) << "Obj" << "\t" <<
				std::left << std::setw(8) << "Step" << "\t" <<
				std::left << std::setw(4) << "LS" << "\t" <<
				std::left << std::setw(8) << "Optimal" << "\t" <<
				std::left << std::setw(8) << "Feasible" << "\t" <<
				std::left << std::setw(14) << "Objective" << "\t" <<
				std::left << std::setw(14) << "Merit" << "\t" <<
				std::left << std::setw(8) << "Scaling" << "\t" <<
				std::left << std::setw(8) << "Penalty" << "\t" <<
				std::left << std::setw(5) << "nI" << "\t" <<
				std::left << std::setw(5) << "mA" << "\t" <<
				std::left << std::setw(9) << "dHd" << "\t" <<
				std::left << std::setw(9) << "dMag" << "\t" <<
				"\n";
		}
		
		// Output data.
		std::cout <<
			std::left << std::setw(5) << std::defaultfloat << problem.kIteration << "\t" <<
			std::left << std::setw(5) << std::defaultfloat << problem.noMinorIterations << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << problem.maxMinorOptimalityViolation_k[0] << "\t" <<
			std::left << std::setw(5) << std::defaultfloat << problem.nofObjEvaluations << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << problem.alphaStepSize_k[0] << "\t" <<
			std::left << std::setw(4) << problem.lineSearchProcedure_k << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << problem.maxOptimalityViolation_k[0] << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << problem.maxConstraintViolation_k[0] << "\t" <<
			std::left << std::setw(14) << std::scientific << std::setprecision(7) << problem.fObjectiveFunction_k[0] << "\t" <<
			std::left << std::setw(14) << std::scientific << std::setprecision(7) << problem.phiLineSearchFunction_k[0] << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << problem.inverseHessianScaling_k[0] << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << pMax << "\t" <<
			std::left << std::setw(5) << std::defaultfloat << problem.nINoInactiveVariables_k << "\t" <<
			std::left << std::setw(5) << std::defaultfloat << (problem.mENoEqualityConstraints + problem.mANoActiveConstraints_k) << "\t" <<
			std::left << std::setw(9) << std::scientific << std::setprecision(1) << dHd << "\t" <<
			std::left << std::setw(9) << std::scientific << std::setprecision(1) << dMag << "\t";

		for (integer i = 0; i < (integer)problem.code.size(); ++i)
		{
			std::cout << problem.code[i] << " ";
		}
		std::cout << "\n";

		// Output headers.
		if (problem.kIteration < 1)
		{
			problem.outputFile <<
				std::left << std::setw(5) << "Maj" << "\t" <<
				std::left << std::setw(5) << "Min " << "\t" <<
				std::left << std::setw(8) << "Conv" << "\t" <<
				std::left << std::setw(5) << "Obj" << "\t" <<
				std::left << std::setw(8) << "Step" << "\t" <<
				std::left << std::setw(4) << "LS" << "\t" <<
				std::left << std::setw(8) << "Optimal" << "\t" <<
				std::left << std::setw(8) << "Feasible" << "\t" <<
				std::left << std::setw(14) << "Objective" << "\t" <<
				std::left << std::setw(14) << "Merit" << "\t" <<
				std::left << std::setw(8) << "Scaling" << "\t" <<
				std::left << std::setw(8) << "Penalty" << "\t" <<
				std::left << std::setw(5) << "nI" << "\t" <<
				std::left << std::setw(5) << "mA" << "\t" <<
				std::left << std::setw(9) << "dHd" << "\t" <<
				std::left << std::setw(9) << "dMag" << "\t" <<
				"\n";
		}

		// Output data.
		problem.outputFile <<
			std::left << std::setw(5) << std::defaultfloat << problem.kIteration << "\t" <<
			std::left << std::setw(5) << std::defaultfloat << problem.noMinorIterations << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << problem.maxMinorOptimalityViolation_k[0] << "\t" <<
			std::left << std::setw(5) << std::defaultfloat << problem.nofObjEvaluations << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << problem.alphaStepSize_k[0] << "\t" <<
			std::left << std::setw(4) << problem.lineSearchProcedure_k << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << problem.maxOptimalityViolation_k[0] << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << problem.maxConstraintViolation_k[0] << "\t" <<
			std::left << std::setw(14) << std::scientific << std::setprecision(7) << problem.fObjectiveFunction_k[0] << "\t" <<
			std::left << std::setw(14) << std::scientific << std::setprecision(7) << problem.phiLineSearchFunction_k[0] << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << problem.inverseHessianScaling_k[0] << "\t" <<
			std::left << std::setw(8) << std::scientific << std::setprecision(1) << pMax << "\t" <<
			std::left << std::setw(5) << std::defaultfloat << problem.nINoInactiveVariables_k << "\t" <<
			std::left << std::setw(5) << std::defaultfloat << (problem.mENoEqualityConstraints + problem.mANoActiveConstraints_k) << "\t" <<
			std::left << std::setw(9) << std::scientific << std::setprecision(1) << dHd << "\t" <<
			std::left << std::setw(9) << std::scientific << std::setprecision(1) << dMag << "\t";

		for (integer i = 0; i < (integer)problem.code.size(); ++i)
		{
			problem.outputFile << problem.code[i] << " ";
		}
		problem.outputFile << "\n";

		problem.code.clear();

		// Set intermediate time 2.
		//problem.intermediateTime2 = highresolutionclock::now();

		// Calculate output seconds.
		//problem.outputSeconds += (std::chrono::duration_cast<std::chrono::milliseconds>(problem.intermediateTime2 - problem.intermediateTime1).count()) / 1000.0;

	}
}

// Function that outputs optimization problem solution.
void LASO::SolverAL::outputProblemSolution(ProblemAL &problem)
{
	// If output set to true.
	if (problem.options.output)
	{
		//problem.lasoSeconds = (std::chrono::duration_cast<std::chrono::milliseconds>(problem.finishTime - problem.startTime).count()) / 1000.0 -
		//	problem.outputSeconds - problem.objectiveSeconds - problem.gradientSeconds - problem.constraintSeconds - problem.jacobianSeconds;

		//std::cout << "\n";
		//std::cout << "\n";
		//std::cout << std::left << std::setw(40) << "Time for solving problem" << std::defaultfloat << std::setprecision(4) << problem.lasoSeconds << " seconds\n";
		//std::cout << std::left << std::setw(40) << "Time for solution output" << std::defaultfloat << std::setprecision(4) << problem.outputSeconds << " seconds\n";
		//std::cout << std::left << std::setw(40) << "Time for objective function" << std::defaultfloat << std::setprecision(4) << problem.objectiveSeconds << " seconds\n";
		//std::cout << std::left << std::setw(40) << "Time for objective gradient" << std::defaultfloat << std::setprecision(4) << problem.gradientSeconds << " seconds\n";
		//std::cout << std::left << std::setw(40) << "Time for constraint functions" << std::defaultfloat << std::setprecision(4) << problem.constraintSeconds << " seconds\n";
		//std::cout << std::left << std::setw(40) << "Time for constraint jacobian" << std::defaultfloat << std::setprecision(4) << problem.jacobianSeconds << " seconds\n";


		problem.solutionFile << "x" << "\n";

		for (integer i = 0; i < problem.nDesignVariables; ++i)
		{
			problem.solutionFile <<
				std::scientific << std::setprecision(16) << problem.xDesignVariables_k[i] << "\n";
		}

		if (problem.mENoEqualityConstraints > 0)
		{
			problem.solutionFile << "\n";
			problem.solutionFile << "mu" << "\n";

			for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
			{
				problem.solutionFile <<
					std::scientific << std::setprecision(16) << problem.muLagrangeMultipliers_k[i] << "\n";
			}
		}

		if (problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL > 0)
		{
			problem.solutionFile << "\n";
			problem.solutionFile << "lambda" << "\n";

			for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
			{
				problem.solutionFile <<
					std::scientific << std::setprecision(16) << problem.lambdaLagrangeMultipliers_k[i] << "\n";
			}
		}
	}
	
}

// Function that evaluates bVector, used by Lagrange multiplier sub-problem.
void LASO::SolverAL::evaluateBVector(integer kIteration,
	vector &hConstraintFunctions, vector &gConstraintFunctions,
	matrix &jhConstraintJacobian, matrix &jgConstraintJacobian,
	vector &gradientX, matrix &sVectors, matrix &yVectors,
	vectorinteger &variables, integer noVariables,
	vectorinteger &activeConstraints, integer noActiveConstraints,
	vectorinteger &correctionHistory, integer noCorrections,
	vector &bVector, ProblemAL &problem)
{
	// If constrained problem.
	if (problem.mENoEqualityConstraints + noActiveConstraints > 0)
	{
		// Resize bVector.
		bVector.resize(problem.mENoEqualityConstraints + noActiveConstraints);

		// If first iteration, take H as identity.
		if (kIteration < 1)
		{
			// Evaluate qVector
			for (integer i = 0; i < noVariables; ++i)
			{
				problem.qVector[variables[i]] = gradientX[variables[i]];
			}

			// Evaluate bVector
			for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
			{
				bVector[i] = hConstraintFunctions[i];

				for (integer j = 0; j < noVariables; ++j)
				{
					bVector[i] -= jhConstraintJacobian[i][variables[j]] * problem.qVector[variables[j]];
				}
			}
			for (integer i = problem.mENoEqualityConstraints; i < problem.mENoEqualityConstraints + noActiveConstraints; ++i)
			{
				bVector[i] = gConstraintFunctions[activeConstraints[i - problem.mENoEqualityConstraints]];

				for (integer j = 0; j < noVariables; ++j)
				{
					bVector[i] -= jgConstraintJacobian[activeConstraints[i - problem.mENoEqualityConstraints]][variables[j]] * problem.qVector[variables[j]];
				}
			}
		}
		// Else take H as l-bfgs approximation of the inverse Hessian of the Lagrangian function.
		else
		{
			// Evaluate qVector
			for (integer i = 0; i < noVariables; ++i)
			{
				problem.qVector[variables[i]] = gradientX[variables[i]];
			}

			// Evaluate rVector = Hq
			LASO::RecursionLBFGS recursionLBFGS;
			recursionLBFGS.evaluateRVector(sVectors, yVectors,
				problem.qVector, variables, noVariables,
				kIteration, noCorrections, correctionHistory,
				problem.qVector, problem.inverseHessianScaling_k, problem.rVector);

			// Evaluate bVector
			for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
			{
				bVector[i] = hConstraintFunctions[i];

				for (integer j = 0; j < noVariables; ++j)
				{
					bVector[i] -= jhConstraintJacobian[i][variables[j]] * problem.rVector[variables[j]];
				}
			}
			for (integer i = problem.mENoEqualityConstraints; i < problem.mENoEqualityConstraints + noActiveConstraints; ++i)
			{
				bVector[i] = gConstraintFunctions[activeConstraints[i - problem.mENoEqualityConstraints]];

				for (integer j = 0; j < noVariables; ++j)
				{
					bVector[i] -= jgConstraintJacobian[activeConstraints[i - problem.mENoEqualityConstraints]][variables[j]] * problem.rVector[variables[j]];
				}
			}
		}
	}
}

// Function that evaluates BMatrix, used by Lagrange multiplier sub-problem.
void LASO::SolverAL::evaluateBMatrix(integer kIteration,
	matrix &jhConstraintJacobian, matrix &jgConstraintJacobian,
	matrix &sVectors, matrix &yVectors,
	vectorinteger &variables, integer noVariables,
	vectorinteger &activeConstraints, integer noActiveConstraints,
	vectorinteger &correctionHistory, integer noCorrections,
	matrix &BMatrix, ProblemAL &problem)
{
	// If constrained problem.
	if (problem.mENoEqualityConstraints + noActiveConstraints > 0)
	{
		// Resize BMatrix.
		BMatrix.resize(problem.mENoEqualityConstraints + noActiveConstraints);
		for (int i = 0; i < problem.mENoEqualityConstraints + noActiveConstraints; ++i)
		{
			BMatrix[i].resize(problem.mENoEqualityConstraints + noActiveConstraints);
		}

		// If first iteration, take H as identity.
		if (kIteration < 1)
		{
			// Equality constraints.
			for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
			{
				for (integer j = 0; j < problem.mENoEqualityConstraints; ++j)
				{
					BMatrix[i][j] = 0.0;
					for (integer k = 0; k < noVariables; ++k)
					{
						BMatrix[i][j] += jhConstraintJacobian[i][variables[k]] * jhConstraintJacobian[j][variables[k]];
					}
				}
				for (integer j = problem.mENoEqualityConstraints; j < problem.mENoEqualityConstraints + noActiveConstraints; ++j)
				{
					BMatrix[i][j] = 0.0;
					for (integer k = 0; k < noVariables; ++k)
					{
						BMatrix[i][j] += jhConstraintJacobian[i][variables[k]] * jgConstraintJacobian[activeConstraints[j - problem.mENoEqualityConstraints]][variables[k]];
					}
				}
			}
			// Inequality constraints.
			for (integer i = problem.mENoEqualityConstraints; i < problem.mENoEqualityConstraints + noActiveConstraints; ++i)
			{
				for (integer j = 0; j < problem.mENoEqualityConstraints; ++j)
				{
					BMatrix[i][j] = 0.0;
					for (integer k = 0; k < noVariables; ++k)
					{
						BMatrix[i][j] += jgConstraintJacobian[activeConstraints[i - problem.mENoEqualityConstraints]][variables[k]] * jhConstraintJacobian[j][variables[k]];
					}
				}
				for (integer j = problem.mENoEqualityConstraints; j < problem.mENoEqualityConstraints + noActiveConstraints; ++j)
				{
					BMatrix[i][j] = 0.0;
					for (integer k = 0; k < noVariables; ++k)
					{
						BMatrix[i][j] += jgConstraintJacobian[activeConstraints[i - problem.mENoEqualityConstraints]][variables[k]] * jgConstraintJacobian[activeConstraints[j - problem.mENoEqualityConstraints]][variables[k]];
					}
				}
			}
		}
		// Else take H as l-bfgs approximation of the inverse Hessian of the augmented Lagrangian function.
		else
		{
			// Equality constraints.
			for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
			{
				// Evaluate rVector = H * Jh[i]
				LASO::RecursionLBFGS recursionLBFGS;
				recursionLBFGS.evaluateRVector(sVectors, yVectors,
					jhConstraintJacobian[i], variables, noVariables,
					kIteration, noCorrections, correctionHistory,
					problem.qVector, problem.inverseHessianScaling_k, problem.rVector);

				for (integer j = 0; j < problem.mENoEqualityConstraints; ++j)
				{
					BMatrix[i][j] = 0.0;
					for (integer k = 0; k < noVariables; ++k)
					{
						BMatrix[i][j] += jhConstraintJacobian[j][variables[k]] * problem.rVector[variables[k]];
					}
				}
				for (integer j = problem.mENoEqualityConstraints; j < problem.mENoEqualityConstraints + noActiveConstraints; ++j)
				{
					BMatrix[i][j] = 0.0;
					for (integer k = 0; k < noVariables; ++k)
					{
						BMatrix[i][j] += jgConstraintJacobian[activeConstraints[j - problem.mENoEqualityConstraints]][variables[k]] * problem.rVector[variables[k]];
					}
				}
			}
			// Inequality constraints.
			for (integer i = problem.mENoEqualityConstraints; i < problem.mENoEqualityConstraints + noActiveConstraints; ++i)
			{
				// Evaluate rVector = H * Jg[i]
				LASO::RecursionLBFGS recursionLBFGS;
				recursionLBFGS.evaluateRVector(sVectors, yVectors,
					jgConstraintJacobian[activeConstraints[i - problem.mENoEqualityConstraints]], variables, noVariables,
					kIteration, noCorrections, correctionHistory,
					problem.qVector, problem.inverseHessianScaling_k, problem.rVector);

				for (integer j = 0; j < problem.mENoEqualityConstraints; ++j)
				{
					BMatrix[i][j] = 0.0;
					for (integer k = 0; k < noVariables; ++k)
					{
						BMatrix[i][j] += jhConstraintJacobian[j][variables[k]] * problem.rVector[variables[k]];
					}
				}
				for (integer j = problem.mENoEqualityConstraints; j < problem.mENoEqualityConstraints + noActiveConstraints; ++j)
				{
					BMatrix[i][j] = 0.0;
					for (integer k = 0; k < noVariables; ++k)
					{
						BMatrix[i][j] += jgConstraintJacobian[activeConstraints[j - problem.mENoEqualityConstraints]][variables[k]] * problem.rVector[variables[k]];
					}
				}
			}
		}
	}
}

// Function that evaluates the Lagrange multiplier search direction.
void LASO::SolverAL::evaluateLagrangeMultiplierSearchDirection(matrix &BMatrix, vector &bVector,
	vectorinteger &activeConstraints, integer noActiveConstraints,
	vector &dmuSearchDirection, vector &dlambdaSearchDirection, ProblemAL &problem)
{
	// If constrained problem.
	if (problem.mENoEqualityConstraints + noActiveConstraints > 0)
	{
		// Resize lagrangeMultiplierSearchDirection.
		problem.lagrangeMultiplierSearchDirection.resize(problem.mENoEqualityConstraints + noActiveConstraints);

		// Setup and solve subproblem.
		integer n = problem.mENoEqualityConstraints + noActiveConstraints;
		integer m = 0;
		for (integer i = 0; i < n; ++i)
		{
			problem.lagrangeMultiplierSearchDirection[i] = 0.0;
		}
		LASO::FunctionsLM functionsLM(BMatrix, bVector, n, m);
		vector lb(n, -std::numeric_limits<double>::infinity());
		vector ub(n, std::numeric_limits<double>::infinity());
		LASO::Options options;
		//options.output = true;
		options.majorIterationsLimit = problem.options.minorIterationsLimit;
		options.majorOptimalityTolerance = problem.options.minorOptimalityTolerance;
		options.majorFeasibilityTolerance = problem.options.minorFeasibilityTolerance;
		vector fObj(1);
		vector gObj(n);
		vector fCon(1);
		vector gCon(1);
		LASO::ProblemAL problemQP(n, functionsLM, &problem.lagrangeMultiplierSearchDirection[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		LASO::SolverAL solver;
		solver.solve(problemQP);
		problem.noMinorIterations = problemQP.kIteration;
		problem.maxMinorOptimalityViolation_k[0] = problemQP.maxOptimalityViolation_k[0];

		// Set equality and active inequality Lagrange multiplier search direction components.
		for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
		{
			dmuSearchDirection[i] = problem.lagrangeMultiplierSearchDirection[i];
		}
		for (integer i = 0; i < noActiveConstraints; ++i)
		{
			dlambdaSearchDirection[activeConstraints[i]] = problem.lagrangeMultiplierSearchDirection[problem.mENoEqualityConstraints + i];
		}
	}


	//// If constrained problem.
	//if (problem.mENoEqualityConstraints + noActiveConstraints > 0)
	//{
	//	boost::numeric::ublas::matrix<doublereal> B(problem.mENoEqualityConstraints + noActiveConstraints, problem.mENoEqualityConstraints + noActiveConstraints);
	//	boost::numeric::ublas::vector<doublereal> b(problem.mENoEqualityConstraints + noActiveConstraints);

	//	for (integer i = 0; i < problem.mENoEqualityConstraints + noActiveConstraints; ++i)
	//	{
	//		for (integer j = 0; j < problem.mENoEqualityConstraints + noActiveConstraints; ++j)
	//		{
	//			B(i, j) = BMatrix[i][j];
	//		}
	//		b(i) = bVector[i];
	//	}

	//	boost::numeric::ublas::permutation_matrix<size_t> pm(B.size1());
	//	boost::numeric::ublas::matrix<doublereal> M = B;
	//	boost::numeric::ublas::vector<doublereal> x = b;
	//	boost::numeric::ublas::lu_factorize(M, pm);
	//	boost::numeric::ublas::lu_substitute(M, pm, x);

	//	// Set equality and active inequality Lagrange multiplier search direction components.
	//	for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
	//	{
	//		dmuSearchDirection[i] = x[i];
	//	}
	//	for (integer i = 0; i < noActiveConstraints; ++i)
	//	{
	//		dlambdaSearchDirection[activeConstraints[i]] = x[problem.mENoEqualityConstraints + i];
	//	}
	//}


	//// If constrained problem.
	//if (problem.mENoEqualityConstraints + noActiveConstraints > 0)
	//{
	//	Eigen::MatrixXd B(problem.mENoEqualityConstraints + noActiveConstraints, problem.mENoEqualityConstraints + noActiveConstraints);
	//	Eigen::VectorXd b(problem.mENoEqualityConstraints + noActiveConstraints);

	//	for (integer i = 0; i < problem.mENoEqualityConstraints + noActiveConstraints; ++i)
	//	{
	//		for (integer j = 0; j < problem.mENoEqualityConstraints + noActiveConstraints; ++j)
	//		{
	//			B(i, j) = BMatrix[i][j];
	//		}
	//		b(i) = bVector[i];
	//	}

	//	Eigen::VectorXd x = B.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	//	// Set equality and active inequality Lagrange multiplier search direction components.
	//	for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
	//	{
	//		dmuSearchDirection[i] = x[i];
	//	}
	//	for (integer i = 0; i < noActiveConstraints; ++i)
	//	{
	//		dlambdaSearchDirection[activeConstraints[i]] = x[problem.mENoEqualityConstraints + i];
	//	}

	//	// Calculate convergence
	//	Eigen::VectorXd g(problem.mENoEqualityConstraints + noActiveConstraints);
	//	LASO::FunctionsLM functionsLM(BMatrix, bVector, problem.mENoEqualityConstraints + noActiveConstraints, 0);
	//	functionsLM.objGrad(&x[0], &g[0]);

	//	problem.maxMinorOptimalityViolation_k[0] = g.lpNorm<Eigen::Infinity>();
	//}

}

// Function that evaluates the QP Lagrange multipliers.
void LASO::SolverAL::evaluateLagrangeMultipliersQP(
	vector &muLagrangeMultipliers, vector &lambdaLagrangeMultipliers,
	vector &dmuSearchDirection, vector &dlambdaSearchDirection,
	vector &muLagrangeMultipliersQP, vector &lambdaLagrangeMultipliersQP,
	ProblemAL &problem)
{
	for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
	{
		muLagrangeMultipliersQP[i] = muLagrangeMultipliers[i] + dmuSearchDirection[i];
	}
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		lambdaLagrangeMultipliersQP[i] = lambdaLagrangeMultipliers[i] + dlambdaSearchDirection[i];
	}
}

// Function that evaluates the inactive set l-bfgs design variable search direction, dx.
void LASO::SolverAL::evaluateInactiveDesignVariableSearchDirection(integer kIteration,
	vector &gradient, matrix &sVectors, matrix &yVectors,
	vectorinteger &variables, integer noVariables,
	vectorinteger &correctionHistory, integer noCorrections,
	vector &dxSearchDirection, ProblemAL &problem)
{
	// If first iteration, set design variable search direction as steepest descent direction.
	if (kIteration < 1)
	{
		// Set dxSearchDirection = -gradient * scaling.
		setSearchDirectionToNegAVector(dxSearchDirection, &gradient[0], variables, noVariables, problem.inverseHessianScaling_k[0], problem);
	}
	// Else set design variable search direction as l-bfgs direction.
	else
	{
		// Evaluate rVector = Hg
		LASO::RecursionLBFGS recursionLBFGS;
		recursionLBFGS.evaluateRVector(sVectors, yVectors,
			gradient, variables, noVariables,
			kIteration, noCorrections, correctionHistory,
			problem.qVector, problem.inverseHessianScaling_k, problem.rVector);

		// Set dxSearchDirection = -rVector.
		setSearchDirectionToNegAVector(dxSearchDirection, &problem.rVector[0], variables, noVariables, 1.0, problem);
	}

	// If constrained problem.
	//if (problem.mENoEqualityConstraints + problem.nANoActiveConstraints_k > 0)
	//{
	//	problem.outputFile <<
	//		"dx = " << "{ " <<
	//		dxSearchDirection[0] << ", " <<
	//		dxSearchDirection[1] << " }" <<
	//		"\n";
	//}

}

// Calculate slack variable search direction.
void LASO::SolverAL::evaluateInactiveSlackVariableSearchDirection(vector &dxSearchDirection,
	vector &gConstraintFunctions, matrix &jgConstraintJacobian,
	vectorinteger &inactiveConstraintSet, integer noInactiveConstraints,
	vectorinteger &variables, integer noVariables,
	vector &zSlackVariables, vector &dzSearchDirection, ProblemAL &problem)
{
	for (integer i = 0; i < noInactiveConstraints; ++i)
	{
		dzSearchDirection[inactiveConstraintSet[i]] =
			-gConstraintFunctions[inactiveConstraintSet[i]] -
			zSlackVariables[inactiveConstraintSet[i]];

		for (integer j = 0; j < noVariables; ++j)
		{
			dzSearchDirection[inactiveConstraintSet[i]] -=
				jgConstraintJacobian[inactiveConstraintSet[i]][variables[j]] *
				dxSearchDirection[variables[j]];
		}
	}

	// If constrained problem.
	//if (problem.mENoEqualityConstraints + problem.nANoActiveConstraints_k > 0)
	//{
	//	problem.outputFile <<
	//		"dz = " << "{ " <<
	//		dzSearchDirection[0] << ", " <<
	//		dzSearchDirection[1] << " }" <<
	//		"\n";
	//}
}

// Set dxSearchDirection = -gradient.
void LASO::SolverAL::setSearchDirectionToNegAVector(vector &dxSearchDirection, double *aVector, vectorinteger &components, integer noComponents, doublereal scaling, ProblemAL &problem)
{
	for (integer i = 0; i < noComponents; ++i)
	{
		dxSearchDirection[components[i]] = -aVector[components[i]] * scaling;
	}
}

// Function that evaluates cVector, used by penalty parameter sub-problem.
void LASO::SolverAL::evaluateCVector(vector &lagrangianGradientQP,
	vector &lagrangianGradient, vector &dxSearchDirection,
	vector &lambdaLagrangeMultipliers, vector &dzSearchDirection,
	vector &hConstraintFunctions, vector &dmuSearchDirection,
	vector &gConstraintFunctions, vector &zSlackVariables,
	vector &dlambdaSearchDirection, vector &cVector, ProblemAL &problem)
{
	// Initialize cVector 
	cVector[0] = 0.0;

	// Add 0.5 * lagrangianGradientQP dot dxSearchDirection.
	for (integer i = 0; i < problem.nDesignVariables; ++i)
	{
		cVector[0] += 0.5 * lagrangianGradientQP[i] * dxSearchDirection[i];
	}

	// Subtract lagrangianGradient dot dxSearchDirection.
	for (integer i = 0; i < problem.nDesignVariables; ++i)
	{
		cVector[0] -= lagrangianGradient[i] * dxSearchDirection[i];
	}

	// Subtract lambdaLagrangeMultipliers dot dzSearchDirection.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		cVector[0] -= lambdaLagrangeMultipliers[i] * dzSearchDirection[i];
	}

	// Subtract hConstraintFunctions dot dmuSearchDirection.
	for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
	{
		cVector[0] -= hConstraintFunctions[i] * dmuSearchDirection[i];
	}

	// Subtract (gConstraintFunctions + zSlackVariables) dot dlambdaSearchDirection.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		cVector[0] -= (gConstraintFunctions[i] + zSlackVariables[i]) * dlambdaSearchDirection[i];
	}
}

// Function that evaluates CMatrix, used by penalty parameter sub-problem.
void LASO::SolverAL::evaluateCMatrix(vector &dxSearchDirection, vector &dzSearchDirection,
	matrix &jhConstraintJacobian, vector &hConstraintFunctions,
	matrix &jgConstraintJacobian, vector &gConstraintFunctions, vector &zSlackVariables,
	vector &CMatrix, ProblemAL &problem)
{
	// Equality constraint components.
	for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
	{
		CMatrix[i] = 0.0;
		for (integer j = 0; j < problem.nDesignVariables; ++j)
		{
			CMatrix[i] += jhConstraintJacobian[i][j] * dxSearchDirection[j] * hConstraintFunctions[i];
		}
	}

	// Inequality constraint components.
	for (integer i = problem.mENoEqualityConstraints; i < problem.mENoEqualityConstraints + problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		CMatrix[i] = 0.0;
		for (integer j = 0; j < problem.nDesignVariables; ++j)
		{
			CMatrix[i] += jgConstraintJacobian[i - problem.mENoEqualityConstraints][j] *
				dxSearchDirection[j] * (gConstraintFunctions[i - problem.mENoEqualityConstraints] +
					zSlackVariables[i - problem.mENoEqualityConstraints]);
		}
		CMatrix[i] += dzSearchDirection[i - problem.mENoEqualityConstraints] *
			(gConstraintFunctions[i - problem.mENoEqualityConstraints] +
				zSlackVariables[i - problem.mENoEqualityConstraints]);
	}
}

// Function that updates equality and inequality constraint penalty parameter vectors.
//void LASO::SolverAL::updatePenaltyParameterVectors(vector &CMatrix, vector &cVector, vector &phiPrime0,
//	vector &lagrangianGradientQP, vector &dxSearchDirection,
//	vector &phPenalties, vector &pgPenalties, ProblemAL &problem)
//{
//	// If constrained problem.
//	if (problem.mENoEqualityConstraints + problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL > 0)
//	{
//		// Setup and solve subproblem for intermediate penalties.
//		integer n = problem.mENoEqualityConstraints + problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL + 1;
//		integer m = 0;
//		LASO::FunctionsP functionsP(CMatrix, cVector, n, m);
//		vector lb(n, 0.0);
//		lb[n - 1] = -std::numeric_limits<double>::infinity();
//		vector ub(n, std::numeric_limits<double>::infinity());
//		LASO::Options options;
//		vector fObj(1);
//		vector gObj(n);
//		vector fCon(1);
//		vector gCon(1);
//		LASO::ProblemAL problemP(n, functionsP, &problem.pSolution[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
//		LASO::SolverAL solver;
//		solver.solve(problemP);
//
//		doublereal pDamping = 1.0;
//
//		// Update penalties.
//		for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
//		{
//			if (phPenalties[i] < 4.0 * (problem.pSolution[i] + pDamping))
//			{
//				phPenalties[i] = std::max(problem.pSolution[i], phPenalties[i]);
//			}
//			else
//			{
//				phPenalties[i] = std::max(problem.pSolution[i], sqrt(phPenalties[i] * (problem.pSolution[i] + pDamping)));
//			}
//		}
//		for (integer i = problem.mENoEqualityConstraints; i < problem.mENoEqualityConstraints + problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
//		{
//			if (pgPenalties[i - problem.mENoEqualityConstraints] < 4.0 * (problem.pSolution[i] + pDamping))
//			{
//				pgPenalties[i - problem.mENoEqualityConstraints] = std::max(problem.pSolution[i], pgPenalties[i - problem.mENoEqualityConstraints]);
//			}
//			else
//			{
//				pgPenalties[i - problem.mENoEqualityConstraints] = std::max(problem.pSolution[i], sqrt(pgPenalties[i - problem.mENoEqualityConstraints] * (problem.pSolution[i] + pDamping)));
//			}
//		}
//	}
//}
void LASO::SolverAL::updatePenaltyParameterVectors(vector &CMatrix, vector &cVector, vector &phiPrime0,
	vector &lagrangianGradientQP, vector &dxSearchDirection,
	vector &phPenalties, vector &pgPenalties, ProblemAL &problem)
{
	// If constrained problem.
	if (problem.mENoEqualityConstraints + problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL > 0)
	{
		// Initialize LqpDotdx 
		doublereal LqpDotdx = 0.0;

		// Add lagrangianGradientQP dot dxSearchDirection.
		for (integer i = 0; i < problem.nDesignVariables; ++i)
		{
			LqpDotdx += lagrangianGradientQP[i] * dxSearchDirection[i];
		}

		// If directional derivative not sufficiently negative.
		if (phiPrime0[0] > 0.5 * LqpDotdx)
		{
			// Setup and solve subproblem for intermediate penalties.
			integer n = problem.mENoEqualityConstraints + problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL + 1;
			integer m = 0;
			LASO::FunctionsP functionsP(CMatrix, cVector, n, m);
			vector lb(n, 0.0);
			lb[n - 1] = -std::numeric_limits<double>::infinity();
			vector ub(n, std::numeric_limits<double>::infinity());
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problemP(n, functionsP, &problem.pSolution[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.solve(problemP);

			// Update penalties.
			for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
			{
				phPenalties[i] = std::max(problem.pSolution[i], 2.0 * phPenalties[i]);
			}
			for (integer i = problem.mENoEqualityConstraints; i < problem.mENoEqualityConstraints + problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
			{
				pgPenalties[i - problem.mENoEqualityConstraints] = std::max(problem.pSolution[i], 2.0 * pgPenalties[i - problem.mENoEqualityConstraints]);
			}
		}
	}
}

// Function that ensures calculated search direction is that of descent.
void LASO::SolverAL::ensureInactiveSearchDirectionThatOfDescent(vector &direction,
	vector &gradient, vectorinteger &components, integer noComponents,
	vectorinteger &correctionHistory, ProblemAL &problem)
{
	if (problem.kIterationSinceRestart > 0)
	{
		problem.directionMagnitude = 0.0;
		problem.gradientMagnitude = 0.0;
		problem.directionDotGradient = 0.0;

		// Calculate search direction magnitude.
		for (integer i = 0; i < noComponents; ++i)
		{
			problem.directionMagnitude += direction[components[i]] * direction[components[i]];
		}
		problem.directionMagnitude = sqrt(problem.directionMagnitude);

		// Calculate gradient magnitude.
		for (int i = 0; i < noComponents; ++i)
		{
			problem.gradientMagnitude += gradient[components[i]] * gradient[components[i]];
		}
		problem.gradientMagnitude = sqrt(problem.gradientMagnitude);

		// Calculate dot product of direction and gradient.
		for (int i = 0; i < noComponents; ++i)
		{
			problem.directionDotGradient += direction[components[i]] * gradient[components[i]];
		}

		// If not direction of descent.
		if (problem.sigmaOneFactor * problem.gradientMagnitude > problem.directionMagnitude ||
			problem.directionMagnitude > problem.sigmaTwoFactor * problem.gradientMagnitude ||
			(-problem.directionDotGradient / (problem.directionMagnitude * problem.gradientMagnitude)) <
			(problem.sigmaOneFactor / problem.sigmaTwoFactor))
		{
			// Restart.
			problem.correctionHistory.clear();
			problem.kIterationSinceRestart = 0;

			// Set search direction to that of steepest descent.
			setSearchDirectionToNegAVector(direction, &gradient[0], components, noComponents, problem.inverseHessianScaling_k[0], problem);
		}
	}
}

// Initialize step size.
void LASO::SolverAL::initializeStepSize(vector &alpha,
	vector &zSlackVariables, vector &dzSearchDirection,
	vector &lambdaLagrangeMultipliers, vector &dlambdaSearchDirection,
	ProblemAL &problem)
{
	// Start with 1.0.
	alpha[0] = 1.0;

	doublereal alpha_i = 0.0;

	// Update to smallest step that can be taken such that positivity of slack variables is maintained.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		// Calculate alpha_i = -z_i / dz_i, i.e. solve 0 == z_i + alpha_i * dz_i for alpha_i.
		alpha_i = -zSlackVariables[i] / dzSearchDirection[i];

		// So long as alpha_i is positive, take step size as smaller of alpha_i or current.
		if (alpha_i > 0.0)
		{
			alpha[0] = std::min(alpha_i, alpha[0]);
		}
	}

	// Update to smallest step that can be taken such that positivity of inequality constraint lagrange multipliers is maintained.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		// Calculate alpha_i = -lambda_i / dlambda_i, i.e. solve 0 == lambda_i + alpha_i * dlambda_i for alpha_i.
		alpha_i = -lambdaLagrangeMultipliers[i] / dlambdaSearchDirection[i];

		// So long as alpha_i is positive, take step size as smaller of alpha_i or current.
		if (alpha_i > 0.0)
		{
			alpha[0] = std::min(alpha_i, alpha[0]);
		}
	}

	//problem.outputFile <<
		//std::left << std::setw(8) << std::scientific << std::setprecision(1) << alpha[0] << "\n";

	alpha[0] = 1.0;

}

// Identify step size.
void LASO::SolverAL::identifyStepSize(vector &alpha, integer kIterationSinceRestart,
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
	ProblemAL &problem)
{
	//if (kIterationSinceRestart < 1)
	//{
	//	alpha[0] = 1.0;
	//}
	//else
	//{
	//	alpha[0] = 1.0;
	//}
	integer kLineSearchIteration = 0;
	doublereal alpha0 = 0.0;
	doublereal phi0 = 0.0;
	boolean evaluateFunctions;
	vector alphaValues(0);
	vector phiValues(0);
	vectorboolean skipQI(1);

	// Calculate phi0, i.e. line search function value for step size alpha0.
	evaluateFunctions = false;
	phi0 = returnLineSearchFunction(alpha0,
		xDesignVariables, zSlackVariables,
		muLagrangeMultipliers, lambdaLagrangeMultipliers,
		dxSearchDirection, dzSearchDirection,
		dmuSearchDirection, dlambdaSearchDirection,
		phPenalties, pgPenalties,
		xDesignVariables, zSlackVariables,
		muLagrangeMultipliers, lambdaLagrangeMultipliers,
		evaluateFunctions, fObjectiveFunctionCurrent,
		hConstraintFunctionsCurrent, gConstraintFunctionsCurrent,
		problem);

	// Calculate phi, i.e. line search function value for step size alpha.
	evaluateFunctions = true;
	problem.phiLineSearchFunction_k[0] = returnLineSearchFunction(alpha[0],
		xDesignVariables, zSlackVariables,
		muLagrangeMultipliers, lambdaLagrangeMultipliers,
		dxSearchDirection, dzSearchDirection,
		dmuSearchDirection, dlambdaSearchDirection,
		phPenalties, pgPenalties,
		&xDesignVariablesTrial[0], zSlackVariablesTrial,
		muLagrangeMultipliersTrial, lambdaLagrangeMultipliersTrial,
		evaluateFunctions, &fObjectiveFunctionTrial[0],
		hConstraintFunctionsTrial, gConstraintFunctionsTrial,
		problem);

	// Store step size value for current iteration.
	alphaValues.push_back(alpha[0]);

	// Store line search function value for current iteration.
	phiValues.push_back(problem.phiLineSearchFunction_k[0]);

	// If sufficient decrease condition, i.e. Armijo's rule, satisfied.
	if (testSufficientDecreaseCondition(problem.phiLineSearchFunction_k[0], phi0, alpha[0], phiPrime0, problem) /*|| */
		/*problem.phiLineSearchFunction_k[0] < phi0 */
		/*|| testCurvatureCondition(problem.phiLineSearchFunction[0], phi0, alpha, phiPrime0, problem)*/)
	{
		// Accept step size.
		problem.lineSearchProcedure_k = "N/A";
	}
	else
	{
		do
		{
			// Decrease step size.
			alpha[0] = decreaseStepSize(kLineSearchIteration,
				alphaValues, phiValues, phiPrime0, phi0,
				skipQI, problem);

			// Increment k.
			kLineSearchIteration++;

			// Calculate phi, i.e. line search function value for step size alpha.
			evaluateFunctions = true;
			problem.phiLineSearchFunction_k[0] = returnLineSearchFunction(alpha[0],
				xDesignVariables, zSlackVariables,
				muLagrangeMultipliers, lambdaLagrangeMultipliers,
				dxSearchDirection, dzSearchDirection,
				dmuSearchDirection, dlambdaSearchDirection,
				phPenalties, pgPenalties,
				&xDesignVariablesTrial[0], zSlackVariablesTrial,
				muLagrangeMultipliersTrial, lambdaLagrangeMultipliersTrial,
				evaluateFunctions, &fObjectiveFunctionTrial[0],
				hConstraintFunctionsTrial, gConstraintFunctionsTrial,
				problem);

			// Store step size value for current iteration.
			alphaValues.push_back(alpha[0]);

			// Store line search function value for current iteration.
			phiValues.push_back(problem.phiLineSearchFunction_k[0]);
		}
		// While sufficient decrease condition not satisfied.
		while (!testSufficientDecreaseCondition(problem.phiLineSearchFunction_k[0], phi0, alpha[0], phiPrime0, problem) /*&&*/ 
			/*problem.phiLineSearchFunction_k[0] > phi0 */
			/*&& !testCurvatureCondition(problem.phiLineSearchFunction[0], phi0, alpha, phiPrime0, problem)*/);
																	

		//// If skip quadratic interpolation set to true.
		//if (skipQI[0])
		//{
		//	// Accept step size.
		//	return alpha;
		//}
		//else
		//{
		//	// Perform quadratic interpolation.
		//	doublereal alphaL = alpha0;
		//	doublereal alphaI = alphaValues[kLineSearchIteration];
		//	doublereal alphaU = alphaValues[kLineSearchIteration - 1];
		//	doublereal phiL = phi0;
		//	doublereal phiI = phiValues[kLineSearchIteration];
		//	doublereal phiU = phiValues[kLineSearchIteration - 1];
		//	doublereal a2 = 1.0 / (alphaU - alphaI) *
		//		((phiU - phiL) / (alphaU - alphaL) -
		//		(phiI - phiL) / (alphaI - alphaL));
		//	doublereal a1 = (phiI - phiL) / (alphaI - alphaL) - a2 * (alphaL + alphaI);
		//	doublereal alphaBar = -(a1) / (2.0 * a2);
		//	problem.lineSearchProcedure_k = "QI3";

		//	// Increment k.
		//	kLineSearchIteration++;

		//	// Calculate phi, i.e. line search function value for step size alpha.
		//	evaluateFunctions = true;
		//	problem.phiLineSearchFunction_k[0] = returnLineSearchFunction(alphaBar,
		//		xDesignVariables, zSlackVariables,
		//		muLagrangeMultipliers, lambdaLagrangeMultipliers,
		//		dxSearchDirection, dzSearchDirection,
		//		dmuSearchDirection, dlambdaSearchDirection,
		//		phPenalties, pgPenalties,
		//		xDesignVariablesTrial, zSlackVariablesTrial,
		//		muLagrangeMultipliersTrial, lambdaLagrangeMultipliersTrial,
		//		evaluateFunctions, fObjectiveFunctionTrial,
		//		hConstraintFunctionsTrial, gConstraintFunctionsTrial,
		//		problem);

		//	// Store step size value for current iteration.
		//	alphaValues.push_back(alphaBar);

		//	// Store line search function value for current iteration.
		//	phiValues.push_back(problem.phiLineSearchFunction_k[0]);

		//	// Accept step size.
		//	return alphaBar;
		//}
	}
}

// Function that decreases step size.
doublereal LASO::SolverAL::decreaseStepSize(integer kLineSearchIteration,
	vector &alphaValues, vector &phiValues, vector &phiPrime0, doublereal phi0,
	vectorboolean &skipQI, ProblemAL &problem)
{
	doublereal alpha;

	if (kLineSearchIteration == 0)
	{
		doublereal alphaSBT = decreaseStepSizeSBT(kLineSearchIteration, alphaValues, problem);
		doublereal alphaQI1 = decreaseStepSizeQI1(kLineSearchIteration, alphaValues, phiValues, phiPrime0, phi0, problem);

		if (alphaQI1 > alphaSBT)
		{
			alpha = alphaQI1;
			skipQI[0] = true;
			problem.lineSearchProcedure_k = "QI1";
		}
		else
		{
			alpha = alphaSBT;
			skipQI[0] = false;
			problem.lineSearchProcedure_k = "SBT";
		}
	}
	else if (kLineSearchIteration == 1)
	{
		doublereal alphaSBT = decreaseStepSizeSBT(kLineSearchIteration, alphaValues, problem);
		doublereal alphaQI2 = decreaseStepSizeQI2(kLineSearchIteration, alphaValues, phiValues, phi0, problem);
		doublereal alphaCI1 = decreaseStepSizeCI1(kLineSearchIteration, alphaValues, phiValues, phiPrime0, phi0, problem);

		if (alphaQI2 > alphaSBT || alphaCI1 > alphaSBT)
		{
			if (alphaQI2 > alphaCI1)
			{
				alpha = alphaQI2;
				skipQI[0] = true;
				problem.lineSearchProcedure_k = "QI2";
			}
			else
			{
				alpha = alphaCI1;
				skipQI[0] = true;
				problem.lineSearchProcedure_k = "CI1";
			}
		}
		else
		{
			alpha = alphaSBT;
			skipQI[0] = false;
			problem.lineSearchProcedure_k = "SBT";
		}
	}
	else
	{
		doublereal alphaSBT = decreaseStepSizeSBT(kLineSearchIteration, alphaValues, problem);
		doublereal alphaQI2 = decreaseStepSizeQI2(kLineSearchIteration, alphaValues, phiValues, phi0, problem);
		doublereal alphaCI2 = decreaseStepSizeCI2(kLineSearchIteration, alphaValues, phiValues, phi0, problem);

		if (alphaQI2 > alphaSBT || alphaCI2 > alphaSBT)
		{
			if (alphaQI2 > alphaCI2)
			{
				alpha = alphaQI2;
				skipQI[0] = true;
				problem.lineSearchProcedure_k = "QI2";
			}
			else
			{
				alpha = alphaCI2;
				skipQI[0] = true;
				problem.lineSearchProcedure_k = "CI2";
			}
		}
		else
		{
			alpha = alphaSBT;
			skipQI[0] = false;
			problem.lineSearchProcedure_k = "SBT";
		}
	}

	//alpha = alphaValues[kLineSearchIteration] / problem.etaFactor;

	return alpha;
}

// Function that decreases step size using backtracking.
doublereal LASO::SolverAL::decreaseStepSizeSBT(integer kLineSearchIteration,
	vector &alphaValues, ProblemAL &problem)
{
	return alphaValues[kLineSearchIteration] / pow(problem.etaFactor, 2.0)/*problem.etaFactor*/;
}

// Function that decreases step size using quadratic interpolation 1.
doublereal LASO::SolverAL::decreaseStepSizeQI1(integer kLineSearchIteration,
	vector &alphaValues, vector &phiValues, vector &phiPrime0, doublereal phi0,
	ProblemAL &problem)
{
	doublereal a0 = phi0;
	doublereal a1 = phiPrime0[0];
	doublereal a2 = (phiValues[kLineSearchIteration] - a0 - a1 * alphaValues[kLineSearchIteration]) / pow(alphaValues[kLineSearchIteration], 2.0);

	doublereal alphaQI1 = -a1 / (2.0 * a2);

	if (2.0 * a2 > 0.0 && 0.0 < alphaQI1 && alphaQI1 < alphaValues[kLineSearchIteration])
	{
		return alphaQI1;
	}
	else
	{
		return 0.0;
	}
}

// Function that decreases step size using quadratic interpolation 2.
doublereal LASO::SolverAL::decreaseStepSizeQI2(integer kLineSearchIteration,
	vector &alphaValues, vector &phiValues, doublereal phi0, ProblemAL &problem)
{
	doublereal a2 = 1.0 /
		(alphaValues[kLineSearchIteration - 1] - alphaValues[kLineSearchIteration]) *
		((phiValues[kLineSearchIteration - 1] - phi0) / alphaValues[kLineSearchIteration - 1] -
		(phiValues[kLineSearchIteration] - phi0) / alphaValues[kLineSearchIteration]);

	doublereal a1 = (phiValues[kLineSearchIteration] - phi0) /
		alphaValues[kLineSearchIteration] - a2 * alphaValues[kLineSearchIteration];

	doublereal a0 = phi0;

	doublereal alphaQI2 = -a1 / (2.0 * a2);

	if (2.0 * a2 > 0.0 && 0.0 < alphaQI2 && alphaQI2 < alphaValues[kLineSearchIteration])
	{
		return alphaQI2;
	}
	else
	{
		return 0.0;
	}
}

// Function that decreases step size using cubic interpolation 1.
doublereal LASO::SolverAL::decreaseStepSizeCI1(integer kLineSearchIteration,
	vector &alphaValues, vector &phiValues, vector &phiPrime0, doublereal phi0,
	ProblemAL &problem)
{
	doublereal a0 = phi0;

	doublereal a1 = phiPrime0[0];

	doublereal a2 = (alphaValues[kLineSearchIteration - 1] *
		pow(alphaValues[kLineSearchIteration], 3.0) * a1 +
		pow(alphaValues[kLineSearchIteration], 3.0) *
		(a0 - phiValues[kLineSearchIteration - 1]) -
		pow(alphaValues[kLineSearchIteration - 1], 3.0) *
		(a0 + alphaValues[kLineSearchIteration] * a1 -
			phiValues[kLineSearchIteration])) /
			(pow(alphaValues[kLineSearchIteration - 1], 2.0) *
		(alphaValues[kLineSearchIteration - 1] -
			alphaValues[kLineSearchIteration]) *
				pow(alphaValues[kLineSearchIteration], 2.0));

	doublereal a3 = ((-alphaValues[kLineSearchIteration - 1]) *
		pow(alphaValues[kLineSearchIteration], 2.0) * a1 +
		pow(alphaValues[kLineSearchIteration], 2.0) *
		(-a0 + phiValues[kLineSearchIteration - 1]) +
		pow(alphaValues[kLineSearchIteration - 1], 2.0) *
		(a0 + alphaValues[kLineSearchIteration] * a1 -
			phiValues[kLineSearchIteration])) /
			(pow(alphaValues[kLineSearchIteration - 1], 2.0) *
		(alphaValues[kLineSearchIteration - 1] -
			alphaValues[kLineSearchIteration]) *
				pow(alphaValues[kLineSearchIteration], 2.0));

	doublereal alphaCI1TRIAL1 = (-a2 + sqrt(pow(a2, 2.0) - 3.0 * a1 * a3)) / (3.0 * a3);

	doublereal alphaCI1TRIAL2 = (-a2 - sqrt(pow(a2, 2.0) - 3.0 * a1 * a3)) / (3.0 * a3);

	if ((pow(a2, 2.0) - 3.0 * a1 * a3) >= 0.0 &&
		0.0 < alphaCI1TRIAL1 &&
		alphaCI1TRIAL1 < alphaValues[kLineSearchIteration] &&
		(2.0 * a2 + 6.0 * a3 * alphaCI1TRIAL1) > 0.0)
	{
		alphaCI1TRIAL1 = alphaCI1TRIAL1;
	}
	else
	{
		alphaCI1TRIAL1 = 0.0;
	}

	if ((pow(a2, 2.0) - 3.0 * a1 * a3) >= 0.0 &&
		0.0 < alphaCI1TRIAL2 &&
		alphaCI1TRIAL2 < alphaValues[kLineSearchIteration] &&
		(2.0 * a2 + 6.0 * a3 * alphaCI1TRIAL2) > 0.0)
	{
		alphaCI1TRIAL2 = alphaCI1TRIAL2;
	}
	else
	{
		alphaCI1TRIAL2 = 0.0;
	}

	return std::max(alphaCI1TRIAL1, alphaCI1TRIAL2);
}

// Function that decreases step size using cubic interpolation 2.
doublereal LASO::SolverAL::decreaseStepSizeCI2(integer kLineSearchIteration,
	vector &alphaValues, vector &phiValues, doublereal phi0,
	ProblemAL &problem)
{
	doublereal a0 = phi0;

	doublereal a1 = ((-pow(alphaValues[kLineSearchIteration - 1], 2.0)) *
		(alphaValues[kLineSearchIteration - 1] - alphaValues[kLineSearchIteration - 2]) *
		pow(alphaValues[kLineSearchIteration - 2], 2.0) *
		(a0 - phiValues[kLineSearchIteration]) +
		pow(alphaValues[kLineSearchIteration], 2.0) *
		(pow(alphaValues[kLineSearchIteration - 2], 3.0) *
		(-a0 + phiValues[kLineSearchIteration - 1]) +
			pow(alphaValues[kLineSearchIteration - 1], 3.0) *
			(a0 - phiValues[kLineSearchIteration - 2])) +
		pow(alphaValues[kLineSearchIteration], 3.0) *
		(pow(alphaValues[kLineSearchIteration - 2], 2.0) *
		(a0 - phiValues[kLineSearchIteration - 1]) +
			pow(alphaValues[kLineSearchIteration - 1], 2.0) *
			(-a0 + phiValues[kLineSearchIteration - 2]))) /
			(alphaValues[kLineSearchIteration] *
		(alphaValues[kLineSearchIteration] - alphaValues[kLineSearchIteration - 1]) *
				alphaValues[kLineSearchIteration - 1] *
				(alphaValues[kLineSearchIteration] - alphaValues[kLineSearchIteration - 2]) *
				(alphaValues[kLineSearchIteration - 1] - alphaValues[kLineSearchIteration - 2]) *
				alphaValues[kLineSearchIteration - 2]);

	doublereal a2 = (alphaValues[kLineSearchIteration - 1] *
		alphaValues[kLineSearchIteration - 2] *
		(pow(alphaValues[kLineSearchIteration - 1], 2.0) -
			pow(alphaValues[kLineSearchIteration - 2], 2)) *
			(a0 - phiValues[kLineSearchIteration]) +
		pow(alphaValues[kLineSearchIteration], 3) *
		(alphaValues[kLineSearchIteration - 2] * (-a0 + phiValues[kLineSearchIteration - 1]) +
			alphaValues[kLineSearchIteration - 1] * (a0 - phiValues[kLineSearchIteration - 2])) +
		alphaValues[kLineSearchIteration] * (pow(alphaValues[kLineSearchIteration - 2], 3.0) *
		(a0 - phiValues[kLineSearchIteration - 1]) +
			pow(alphaValues[kLineSearchIteration - 1], 3.0) *
			(-a0 + phiValues[kLineSearchIteration - 2]))) /
			(alphaValues[kLineSearchIteration] *
		(alphaValues[kLineSearchIteration] - alphaValues[kLineSearchIteration - 1]) *
				alphaValues[kLineSearchIteration - 1] * (alphaValues[kLineSearchIteration] -
					alphaValues[kLineSearchIteration - 2]) * (alphaValues[kLineSearchIteration - 1] -
						alphaValues[kLineSearchIteration - 2]) * alphaValues[kLineSearchIteration - 2]);

	doublereal a3 = (-a0 + (alphaValues[kLineSearchIteration] *
		alphaValues[kLineSearchIteration - 2] * (-alphaValues[kLineSearchIteration] +
			alphaValues[kLineSearchIteration - 2]) * phiValues[kLineSearchIteration - 1] +
		pow(alphaValues[kLineSearchIteration - 1], 2.0) *
		(alphaValues[kLineSearchIteration - 2] * phiValues[kLineSearchIteration] -
			alphaValues[kLineSearchIteration] * phiValues[kLineSearchIteration - 2]) +
		alphaValues[kLineSearchIteration - 1] *
		((-pow(alphaValues[kLineSearchIteration - 2], 2.0)) *
			phiValues[kLineSearchIteration] + pow(alphaValues[kLineSearchIteration], 2.0) *
			phiValues[kLineSearchIteration - 2])) /
			((alphaValues[kLineSearchIteration] - alphaValues[kLineSearchIteration - 1]) *
		(alphaValues[kLineSearchIteration] - alphaValues[kLineSearchIteration - 2]) *
				(alphaValues[kLineSearchIteration - 1] - alphaValues[kLineSearchIteration - 2]))) /
				(alphaValues[kLineSearchIteration] * alphaValues[kLineSearchIteration - 1] *
					alphaValues[kLineSearchIteration - 2]);

	doublereal alphaCI2TRIAL1 = (-a2 + sqrt(pow(a2, 2.0) - 3.0 * a1 * a3)) / (3.0 * a3);

	doublereal alphaCI2TRIAL2 = (-a2 - sqrt(pow(a2, 2.0) - 3.0 * a1 * a3)) / (3.0 * a3);

	if ((pow(a2, 2.0) - 3.0 * a1 * a3) >= 0.0 &&
		0.0 < alphaCI2TRIAL1 &&
		alphaCI2TRIAL1 < alphaValues[kLineSearchIteration] &&
		(2.0 * a2 + 6.0 * a3 * alphaCI2TRIAL1) > 0.0)
	{
		alphaCI2TRIAL1 = alphaCI2TRIAL1;
	}
	else
	{
		alphaCI2TRIAL1 = 0.0;
	}

	if ((pow(a2, 2.0) - 3.0 * a1 * a3) >= 0.0 &&
		0.0 < alphaCI2TRIAL2 &&
		alphaCI2TRIAL2 < alphaValues[kLineSearchIteration] &&
		(2.0 * a2 + 6.0 * a3 * alphaCI2TRIAL2) > 0.0)
	{
		alphaCI2TRIAL2 = alphaCI2TRIAL2;
	}
	else
	{
		alphaCI2TRIAL2 = 0.0;
	}

	return std::max(alphaCI2TRIAL1, alphaCI2TRIAL2);
}

// Function that returns line search function value for arbitrary step size.
doublereal LASO::SolverAL::returnLineSearchFunction(doublereal stepSize,
	double *xDesignVariables, vector &zSlackVariables,
	vector &muLagrangeMultipliers, vector &lambdaLagrangeMultipliers,
	vector &dxSearchDirection, vector &dzSearchDirection,
	vector &dmuSearchDirection, vector &dlambdaSearchDirection,
	vector &phPenalties, vector &pgPenalties,
	double *xDesignVariablesTrial, vector &zSlackVariablesTrial,
	vector &muLagrangeMultipliersTrial, vector &lambdaLagrangeMultipliersTrial,
	boolean evaluateFunctions, double *fObjectiveFunctionTrial,
	vector &hConstraintFunctionsTrial, vector &gConstraintFunctionsTrial,
	ProblemAL &problem)
{
	doublereal fLineSearchFunction = 0.0;

	// Set trial design variables.
	for (integer i = 0; i < problem.nDesignVariables; ++i)
	{
		xDesignVariablesTrial[i] = xDesignVariables[i] + stepSize * dxSearchDirection[i];
	}
	applyProjectionOperator(&xDesignVariablesTrial[0], problem.nDesignVariables,
		problem.lbLowerBounds, problem.ubUpperBounds);

	// Set trial slack variables.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		zSlackVariablesTrial[i] = zSlackVariables[i] + stepSize * dzSearchDirection[i];
	}
	// TODO: Delete. Not doing anything so long as initializeStepSize called.
	applyProjectionOperatorConstantBounds(zSlackVariablesTrial,
		problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL,
		0.0, std::numeric_limits<double>::infinity());

	// Set trial equality constraint Lagrange multiplier variables.
	for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
	{
		muLagrangeMultipliersTrial[i] = muLagrangeMultipliers[i] + stepSize * dmuSearchDirection[i];
	}

	// Set trial inequality constraint Lagrange multiplier variables.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		lambdaLagrangeMultipliersTrial[i] = lambdaLagrangeMultipliers[i] + stepSize * dlambdaSearchDirection[i];
	}
	// TODO: Delete. Not doing anything so long as initializeStepSize called.
	applyProjectionOperatorConstantBounds(lambdaLagrangeMultipliersTrial,
		problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL,
		0.0, std::numeric_limits<double>::infinity());

	fLineSearchFunction = returnAugmentedLagrangianFunction(
		xDesignVariablesTrial, zSlackVariablesTrial,
		muLagrangeMultipliersTrial, lambdaLagrangeMultipliersTrial,
		phPenalties, pgPenalties,
		evaluateFunctions, &fObjectiveFunctionTrial[0],
		hConstraintFunctionsTrial, gConstraintFunctionsTrial,
		problem);

	return fLineSearchFunction;
}

// Function that returns augmented Lagrangian function value.
doublereal LASO::SolverAL::returnAugmentedLagrangianFunction(
	double *xDesignVariablesTrial, vector &zSlackVariablesTrial,
	vector &muLagrangeMultipliersTrial, vector &lambdaLagrangeMultipliersTrial,
	vector &phPenalties, vector &pgPenalties,
	boolean evaluateFunctions, double *fObjectiveFunctionTrial,
	vector &hConstraintFunctionsTrial, vector &gConstraintFunctionsTrial,
	ProblemAL &problem)
{
	doublereal fAugmentedLagrangianFunction = 0.0;

	// Only evaluate objective and constraint functions if evaluateFunctions is set to true.
	if (evaluateFunctions)
	{
		// Evaluate fObjectiveFunctionTrial at xDesignVariablesTrial.
		//problem.intermediateTime1 = highresolutionclock::now();
		problem.functions.objFun(xDesignVariablesTrial, fObjectiveFunctionTrial);
		//problem.intermediateTime2 = highresolutionclock::now();
		//problem.objectiveSeconds += (std::chrono::duration_cast<std::chrono::milliseconds>(problem.intermediateTime2 - problem.intermediateTime1).count()) / 1000.0;
		problem.nofObjEvaluations++;

		// Evaluate hConstraintFunctionsTrial and gConstraintFunctionsTrial at xDesignVariablesTrial.
		evaluateAuxiliaryConstraints(&xDesignVariablesTrial[0],
			hConstraintFunctionsTrial, gConstraintFunctionsTrial, problem);
	}

	// Initialize augmented Lagrangian function.
	fAugmentedLagrangianFunction = fObjectiveFunctionTrial[0];

	// Add penalties for each equality constraint.
	for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
	{
		fAugmentedLagrangianFunction += muLagrangeMultipliersTrial[i] *
			hConstraintFunctionsTrial[i] + 0.5 * phPenalties[i] *
			pow(hConstraintFunctionsTrial[i], 2.0);
	}

	// Add penalties for each inequality constraint.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		fAugmentedLagrangianFunction += lambdaLagrangeMultipliersTrial[i] *
			(gConstraintFunctionsTrial[i] + zSlackVariablesTrial[i]) + 0.5 * pgPenalties[i] *
			pow(gConstraintFunctionsTrial[i] + zSlackVariablesTrial[i], 2.0);
	}

	return fAugmentedLagrangianFunction;
}

// Function that returns the line search function gradient value for step size of 0.
void LASO::SolverAL::evaluatePhiPrime0LineSearchFunctionGradient(
	vector &gradientX, vector &dxSearchDirection,
	vector &gradientZ, vector &dzSearchDirection,
	vector &gradientMu, vector &dmuSearchDirection,
	vector &gradientLambda, vector &dlambdaSearchDirection,
	vector &phiPrime0, ProblemAL &problem)
{
	phiPrime0[0] = 0.0;

	// Add gradientX dot dxSearchDirection.
	for (integer i = 0; i < problem.nDesignVariables; ++i)
	{
		phiPrime0[0] += gradientX[i] * dxSearchDirection[i];
	}

	// Add gradientZ dot dzSearchDirection.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		phiPrime0[0] += gradientZ[i] * dzSearchDirection[i];
	}

	// Add gradientMu dot dmuSearchDirection.
	for (integer i = 0; i < problem.mENoEqualityConstraints; ++i)
	{
		phiPrime0[0] += gradientMu[i] * dmuSearchDirection[i];
	}

	// Add gradientLambda dot dlambdaSearchDirection.
	for (integer i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		phiPrime0[0] += gradientLambda[i] * dlambdaSearchDirection[i];
	}
}

// Function that returns true if sufficient decrease condition, i.e. Armijo's rule, is satisfied.  False otherwise.  
boolean LASO::SolverAL::testSufficientDecreaseCondition(doublereal phiLineSearchFunction,
	doublereal phi0LineSearchFunction, doublereal alphaStepSize,
	vector &phiPrime0, ProblemAL &problem)
{
	if (phiLineSearchFunction <= phi0LineSearchFunction + problem.rhoFactor * alphaStepSize * phiPrime0[0])
	{
		return true;
	}
	else
	{
		return false;
	}
}

// Function that returns true if modified curvature condition is satisfied.  False otherwise.  
boolean LASO::SolverAL::testCurvatureCondition(doublereal phiLineSearchFunction,
	doublereal phi0LineSearchFunction, doublereal alphaStepSize,
	doublereal phiPrime0LineSearchGradient, ProblemAL &problem)
{
	if (abs((phiLineSearchFunction - phi0LineSearchFunction) / alphaStepSize) <= problem.betaFactor * abs(phiPrime0LineSearchGradient) &&
		(phiLineSearchFunction - phi0LineSearchFunction) / alphaStepSize <= 0.0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

// Update s and y correction vectors.
void LASO::SolverAL::updateCorrectionVectors(double *xDesignVariables_k, vector &xDesignVariables_kPlus1,
	vector &gradientX_k, vector &gradientX_kPlus1,
	vectorinteger &variables, integer noVariables,
	vectorinteger &correctionHistory, integer noCorrections,
	matrix &sVectors, matrix &yVectors,
	ProblemAL &problem)
{
	// Add current correction to correction history.
	correctionHistory.push_back(problem.kIterationSinceRestart - noCorrections *
		(problem.kIterationSinceRestart / noCorrections));

	// Calculate dot product of current s and y.
	doublereal sDotY = 0.0;
	for (int i = 0; i < noVariables; ++i)
	{
		sDotY += (xDesignVariables_kPlus1[variables[i]] - xDesignVariables_k[variables[i]]) *
			(gradientX_kPlus1[variables[i]] - gradientX_k[variables[i]]);
	}

	// Calculate y magnitude.
	doublereal yDotY = 0.0;
	for (int i = 0; i < noVariables; ++i)
	{
		yDotY += (gradientX_kPlus1[variables[i]] - gradientX_k[variables[i]]) *
			(gradientX_kPlus1[variables[i]] - gradientX_k[variables[i]]);
	}

	// If sDotY sufficiently large.
	if (sDotY > 2.2e-16 * yDotY)
	{
		// Update appropriate s and y vectors.
		for (int i = 0; i < problem.nDesignVariables; ++i)
		{
			sVectors[correctionHistory[problem.kIterationSinceRestart]][i] =
				(xDesignVariables_kPlus1[i] - xDesignVariables_k[i]);

			yVectors[correctionHistory[problem.kIterationSinceRestart]][i] =
				(gradientX_kPlus1[i] - gradientX_k[i]);
		}
	}
	else
	{
		// Delete current correction from correction history.
		correctionHistory.pop_back();
		--problem.kIterationSinceRestart;
		problem.code.push_back('n');
	}

}

// Set variables for next iteration.
void LASO::SolverAL::setVariablesForNextIteration(
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
	ProblemAL &problem)
{
	fObjectiveFunction_k[0] = fObjectiveFunction_kPlus1[0];

	for (int i = 0; i < problem.nDesignVariables; ++i)
	{
		xDesignVariables_k[i] = xDesignVariables_kPlus1[i];
		objectiveGradient_k[i] = objectiveGradient_kPlus1[i];
		augmentedLagrangianGradientX_k[i] = augmentedLagrangianGradientX_kPlus1[i];
		lagrangianGradient_k[i] = lagrangianGradient_kPlus1[i];
	}

	for (int i = 0; i < problem.mENoEqualityConstraints; ++i)
	{
		muLagrangeMultipliers_k[i] = muLagrangeMultipliers_kPlus1[i];
		hConstraintFunctions_k[i] = hConstraintFunctions_kPlus1[i];
		augmentedLagrangianGradientMu_k[i] = augmentedLagrangianGradientMu_kPlus1[i];

		for (int j = 0; j < problem.nDesignVariables; ++j)
		{
			jhConstraintJacobian_k[i][j] = jhConstraintJacobian_kPlus1[i][j];
		}
	}

	for (int i = 0; i < problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL; ++i)
	{
		zSlackVariables_k[i] = zSlackVariables_kPlus1[i];
		lambdaLagrangeMultipliers_k[i] = lambdaLagrangeMultipliers_kPlus1[i];
		gConstraintFunctions_k[i] = gConstraintFunctions_kPlus1[i];
		augmentedLagrangianGradientZ_k[i] = augmentedLagrangianGradientZ_kPlus1[i];
		augmentedLagrangianGradientLambda_k[i] = augmentedLagrangianGradientLambda_kPlus1[i];

		for (int j = 0; j < problem.nDesignVariables; ++j)
		{
			jgConstraintJacobian_k[i][j] = jgConstraintJacobian_kPlus1[i][j];
		}
	}
}
