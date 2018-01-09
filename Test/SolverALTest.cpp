#include "CppUnitTest.h"

#include "HS002.h"
#include "HS071.h"
#include "SolverAL.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(LBFGSBTest)
	{

	public:

		doublereal fObj = 0.0;

		TEST_METHOD(applyProjectionOperator)
		{

			integer n = 3;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0, 0.0 };
			vector lb = { -INFINITY, 1.5, -1.0 };
			vector ub = { -3.0, INFINITY, 1.0 };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL lbfgsb;

			lbfgsb.applyProjectionOperator(problem.xDesignVariables_k,
				problem.nDesignVariables, problem.lbLowerBounds, problem.ubUpperBounds);
			Assert::AreEqual(-3.0, problem.xDesignVariables_k[0]);
			Assert::AreEqual(1.5, problem.xDesignVariables_k[1]);
			Assert::AreEqual(0.0, problem.xDesignVariables_k[2]);

		}

		TEST_METHOD(applyProjectionOperatorConstantBounds)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL lbfgsb;
			problem.zSlackVariables_k[0] = -1.0;
			lbfgsb.applyProjectionOperatorConstantBounds(problem.zSlackVariables_k,
				problem.mINoInequalityConstraintsU + problem.mINoInequalityConstraintsL,
				0.0, INFINITY);
			Assert::AreEqual(0.0, problem.zSlackVariables_k[0]);

		}

		TEST_METHOD(evaluateAuxiliaryConstraints)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k, problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			Assert::AreEqual(12.0, problem.hConstraintFunctions_k[0]);
			Assert::AreEqual(0.0, problem.gConstraintFunctions_k[0]);

		}

		//TEST_METHOD(evaluateReformattedConstraints)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	LASO::ProblemAL problem(n, hs071, x, lb, ub, m, options);
		//	LASO::SolverAL solver;
		//	problem.zSlackVariables_k[0] = 1.0;
		//	solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
		//		problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
		//	solver.evaluateReformattedConstraints(problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k, problem.rConstraintFunctions_k, problem);
		//	
		//	Assert::AreEqual(12.0, problem.rConstraintFunctions_k[0]);
		//	Assert::AreEqual(1.0, problem.rConstraintFunctions_k[1]);

		//	Assert::AreEqual((integer)1, problem.nofConEvaluations);

		//}

		TEST_METHOD(evaluateReformattedObjectiveGradient)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			Assert::AreEqual(12.0, problem.objectiveGradient_k[0]);
			Assert::AreEqual(1.0, problem.objectiveGradient_k[1]);
			Assert::AreEqual(2.0, problem.objectiveGradient_k[2]);
			Assert::AreEqual(11.0, problem.objectiveGradient_k[3]);
			Assert::AreEqual((integer)1, problem.nogObjEvaluations);

		}

		TEST_METHOD(evaluateAuxiliaryJacobians)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k, problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			Assert::AreEqual(2.0, problem.jhConstraintJacobian_k[0][0]);
			Assert::AreEqual(10.0, problem.jhConstraintJacobian_k[0][1]);
			Assert::AreEqual(10.0, problem.jhConstraintJacobian_k[0][2]);
			Assert::AreEqual(2.0, problem.jhConstraintJacobian_k[0][3]);

			Assert::AreEqual(-25.0, problem.jgConstraintJacobian_k[0][0]);
			Assert::AreEqual(-5.0, problem.jgConstraintJacobian_k[0][1]);
			Assert::AreEqual(-5.0, problem.jgConstraintJacobian_k[0][2]);
			Assert::AreEqual(-25.0, problem.jgConstraintJacobian_k[0][3]);

		}

		//TEST_METHOD(evaluateReformattedJacobian)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	LASO::ProblemAL problem(n, hs071, x, lb, ub, m, options);
		//	LASO::SolverAL solver;
		//	solver.evaluateReformattedJacobian(problem.xDesignVariables_k, problem.jrConstraintJacobian_k, problem);

		//	Assert::AreEqual(2.0, problem.jrConstraintJacobian_k[0][0]);
		//	Assert::AreEqual(10.0, problem.jrConstraintJacobian_k[0][1]);
		//	Assert::AreEqual(10.0, problem.jrConstraintJacobian_k[0][2]);
		//	Assert::AreEqual(2.0, problem.jrConstraintJacobian_k[0][3]);
		//	Assert::AreEqual(0.0, problem.jrConstraintJacobian_k[0][4]);

		//	Assert::AreEqual(-25.0, problem.jrConstraintJacobian_k[1][0]);
		//	Assert::AreEqual(-5.0, problem.jrConstraintJacobian_k[1][1]);
		//	Assert::AreEqual(-5.0, problem.jrConstraintJacobian_k[1][2]);
		//	Assert::AreEqual(-25.0, problem.jrConstraintJacobian_k[1][3]);
		//	Assert::AreEqual(1.0, problem.jrConstraintJacobian_k[1][4]);

		//	Assert::AreEqual((integer)1, problem.nojConEvaluations);

		//}

		TEST_METHOD(calculateMaxFeasibilityViolation)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			//solver.evaluateReformattedConstraints(problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k, problem.rConstraintFunctions_k, problem);
			solver.calculateMaxFeasibilityViolation(problem.hConstraintFunctions_k,
				problem.gConstraintFunctions_k, problem.lambdaLagrangeMultipliers_k,
				problem.maxConstraintViolation_k, problem);

			Assert::AreEqual(12.0, problem.constraintViolations_k[0]);
			Assert::AreEqual(0.0, problem.constraintViolations_k[1]);

			Assert::AreEqual(1.4614358943359949, problem.maxConstraintViolation_k[0]);

		}

		TEST_METHOD(evaluateReformattedLagrangianGradient)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			// Evaluate reformatted objective gradient.
			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			// Evaluate reformatted constraint Jacobian, Jr(x).
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			problem.muLagrangeMultipliers_k[0] = 2.0;
			problem.lambdaLagrangeMultipliers_k[0] = 1.0;
			solver.evaluateLagrangianGradient(problem.objectiveGradient_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.lagrangianGradient_k, problem);
			Assert::AreEqual(-9.0, problem.lagrangianGradient_k[0]);
			Assert::AreEqual(16.0, problem.lagrangianGradient_k[1]);
			Assert::AreEqual(17.0, problem.lagrangianGradient_k[2]);
			Assert::AreEqual(-10.0, problem.lagrangianGradient_k[3]);

		}

		TEST_METHOD(calculateMaxOptimalityViolation)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			problem.muLagrangeMultipliers_k[0] = 2.0;
			problem.lambdaLagrangeMultipliers_k[0] = 1.0;
			solver.evaluateLagrangianGradient(problem.objectiveGradient_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.lagrangianGradient_k, problem);
			solver.calculateMaxOptimalityViolation(problem.lagrangianGradient_k,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.maxOptimalityViolation_k, problem);

			Assert::AreEqual(9.0, problem.optimalityViolations_k[0]);
			Assert::AreEqual(16.0, problem.optimalityViolations_k[1]);
			Assert::AreEqual(17.0, problem.optimalityViolations_k[2]);
			Assert::AreEqual(10.0, problem.optimalityViolations_k[3]);
			//Assert::AreEqual(1.0, problem.optimalityViolations_k[4]);

			Assert::AreEqual(17.000000000000000, problem.maxOptimalityViolation_k[0]);

		}

		TEST_METHOD(testFeasible)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;

			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			//solver.evaluateReformattedConstraints(problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k, problem.rConstraintFunctions_k, problem);
			solver.calculateMaxFeasibilityViolation(problem.hConstraintFunctions_k,
				problem.gConstraintFunctions_k, problem.lambdaLagrangeMultipliers_k,
				problem.maxConstraintViolation_k, problem);

			Assert::AreEqual(false, solver.testFeasible(problem.maxConstraintViolation_k, problem));

			problem.maxConstraintViolation_k[0] = 0.0;

			Assert::AreEqual(true, solver.testFeasible(problem.maxConstraintViolation_k, problem));

		}

		TEST_METHOD(testOptimal)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;

			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			problem.muLagrangeMultipliers_k[0] = 2.0;
			problem.lambdaLagrangeMultipliers_k[0] = 1.0;
			solver.evaluateLagrangianGradient(problem.objectiveGradient_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.lagrangianGradient_k, problem);
			solver.calculateMaxOptimalityViolation(problem.lagrangianGradient_k,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.maxOptimalityViolation_k, problem);

			Assert::AreEqual(false, solver.testOptimal(problem.maxOptimalityViolation_k, problem));

			problem.maxOptimalityViolation_k[0] = 0.0;

			Assert::AreEqual(true, solver.testOptimal(problem.maxOptimalityViolation_k, problem));

		}

		TEST_METHOD(testIterations)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;

			Assert::AreEqual(false, solver.testIterations(problem.kIteration, problem));

			problem.kIteration = 500;

			Assert::AreEqual(true, solver.testIterations(problem.kIteration, problem));

		}

		TEST_METHOD(evaluateAugmentedLagrangianGradients)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			//solver.evaluateReformattedConstraints(problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k,
			//	problem.rConstraintFunctions_k, problem);
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			problem.muLagrangeMultipliers_k[0] = 2.0;
			problem.lambdaLagrangeMultipliers_k[0] = 1.0;
			problem.phPenalties_k[0] = 1.0;
			problem.pgPenalties_k[0] = 1.0;
			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
				problem);

			Assert::AreEqual(15.0, problem.augmentedLagrangianGradientX_k[0]);
			Assert::AreEqual(136.0, problem.augmentedLagrangianGradientX_k[1]);
			Assert::AreEqual(137.0, problem.augmentedLagrangianGradientX_k[2]);
			Assert::AreEqual(14.0, problem.augmentedLagrangianGradientX_k[3]);

			Assert::AreEqual(1.0, problem.augmentedLagrangianGradientZ_k[0]);

			Assert::AreEqual(12.0, problem.augmentedLagrangianGradientMu_k[0]);

			Assert::AreEqual(0.0, problem.augmentedLagrangianGradientLambda_k[0]);

			problem.lambdaLagrangeMultipliers_k[0] = 2.0;
			problem.pgPenalties_k[0] = 3.0;
			problem.zSlackVariables_k[0] = 1.0;

			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			//solver.evaluateReformattedConstraints(problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k,
			//	problem.rConstraintFunctions_k, problem);
			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
				problem);

			Assert::AreEqual(5.0, problem.augmentedLagrangianGradientZ_k[0]);

		}

		TEST_METHOD(identifyActiveInactiveVariableSet)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			//solver.evaluateReformattedConstraints(problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k,
			//	problem.rConstraintFunctions_k, problem);
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			problem.muLagrangeMultipliers_k[0] = 2.0;
			problem.lambdaLagrangeMultipliers_k[0] = 1.0;
			problem.phPenalties_k[0] = 1.0;
			problem.pgPenalties_k[0] = 1.0;
			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
				problem);
			solver.identifyActiveInactiveVariableSet(problem.xDesignVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k,
				problem.inactiveVariableSet_k, problem);
			solver.estimateActiveInactiveConstraintSet(problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
				problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
				problem);

			Assert::AreEqual(0.0, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(0.0, problem.dxSearchDirection_k[1]);
			Assert::AreEqual(0.0, problem.dxSearchDirection_k[2]);
			Assert::AreEqual(0.0, problem.dxSearchDirection_k[3]);

			Assert::AreEqual(0.0, problem.dzSearchDirection_k[0]);

			Assert::AreEqual(15.0, problem.lambdaLagrangeMultipliersL_k[0]);
			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersL_k[1]);
			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersL_k[2]);
			Assert::AreEqual(14.0, problem.lambdaLagrangeMultipliersL_k[3]);
			Assert::AreEqual(1.0, problem.lambdaLagrangeMultipliersL_k[4]);

			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersU_k[0]);
			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersU_k[1]);
			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersU_k[2]);
			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersU_k[3]);
			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersU_k[4]);

			Assert::AreEqual((integer)1, problem.inactiveVariableSet_k[0]);
			Assert::AreEqual((integer)2, problem.inactiveVariableSet_k[1]);

			Assert::AreEqual((integer)2, problem.nINoInactiveVariables_k);

			Assert::AreEqual((integer)0, problem.activeConstraintSet_k[0]);

			Assert::AreEqual((integer)1, problem.mANoActiveConstraints_k);

			problem.options.majorFeasibilityTolerance = 0.1;
			problem.xDesignVariables_k[0] = 1.0 + 0.05;
			problem.augmentedLagrangianGradientX_k[0] = 15;
			problem.xDesignVariables_k[1] = 5.0 - 0.05;
			problem.augmentedLagrangianGradientX_k[1] = -0.025;
			problem.xDesignVariables_k[2] = 5.0 - 0.05;
			problem.augmentedLagrangianGradientX_k[2] = -137.0;
			problem.xDesignVariables_k[3] = 1.0 + 0.05;
			problem.augmentedLagrangianGradientX_k[3] = 0.025;
			problem.zSlackVariables_k[0] = 0.05;
			problem.augmentedLagrangianGradientZ_k[0] = 0.01;
			solver.identifyActiveInactiveVariableSet(problem.xDesignVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k,
				problem.inactiveVariableSet_k, problem);
			solver.estimateActiveInactiveConstraintSet(problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
				problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
				problem);

			Assert::AreEqual(-0.050000000000000044, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(0.025, problem.dxSearchDirection_k[1]);
			Assert::AreEqual(0.049999999999999822, problem.dxSearchDirection_k[2]);
			Assert::AreEqual(-0.025, problem.dxSearchDirection_k[3]);

			Assert::AreEqual(-0.01, problem.dzSearchDirection_k[0]);

			Assert::AreEqual(15.0, problem.lambdaLagrangeMultipliersL_k[0]);
			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersL_k[1]);
			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersL_k[2]);
			Assert::AreEqual(0.025, problem.lambdaLagrangeMultipliersL_k[3]);
			Assert::AreEqual(0.01, problem.lambdaLagrangeMultipliersL_k[4]);

			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersU_k[0]);
			Assert::AreEqual(0.025, problem.lambdaLagrangeMultipliersU_k[1]);
			Assert::AreEqual(137.0, problem.lambdaLagrangeMultipliersU_k[2]);
			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersU_k[3]);
			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliersU_k[4]);

			Assert::AreEqual((integer)0, problem.activeConstraintSet_k[0]);

			Assert::AreEqual((integer)1, problem.mANoActiveConstraints_k);
			Assert::AreEqual((integer)0, problem.nINoInactiveVariables_k);

		}

		TEST_METHOD(evaluateBVector)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			problem.phPenalties_k[0] = 1.0;
			problem.pgPenalties_k[0] = 1.0;
			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
				problem);
			solver.identifyActiveInactiveVariableSet(problem.xDesignVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k,
				problem.inactiveVariableSet_k, problem);
			solver.estimateActiveInactiveConstraintSet(problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
				problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
				problem);
			solver.evaluateBVector(problem.kIterationSinceRestart,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.augmentedLagrangianGradientX_k, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.bVector_k, problem);

			Assert::AreEqual(-2418.0, problem.bVector_k[0]);
			//Assert::AreEqual(1215.0, problem.bVector_k[1]);

			solver.evaluateInactiveDesignVariableSearchDirection(problem.kIterationSinceRestart,
				problem.augmentedLagrangianGradientX_k, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);
			solver.ensureInactiveSearchDirectionThatOfDescent(problem.dxSearchDirection_k,
				problem.augmentedLagrangianGradientX_k, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem);
			solver.evaluatePhiPrime0LineSearchFunctionGradient(
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k,
				problem.augmentedLagrangianGradientMu_k, problem.dmuSearchDirection_k,
				problem.augmentedLagrangianGradientLambda_k, problem.dlambdaSearchDirection_k,
				problem.phiPrime0_k, problem);
			solver.initializeStepSize(problem.alphaStepSize_k,
				problem.zSlackVariables_k, problem.dzSearchDirection_k,
				problem.lambdaLagrangeMultipliers_k, problem.dlambdaSearchDirection_k,
				problem);
			solver.identifyStepSize(problem.alphaStepSize_k, problem.kIterationSinceRestart,
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
			solver.evaluateObjectiveGradient(&problem.xDesignVariables_kPlus1[0],
				&problem.objectiveGradient_kPlus1[0], problem);
			solver.evaluateAuxiliaryJacobians(&problem.xDesignVariables_kPlus1[0],
				problem.jhConstraintJacobian_kPlus1, problem.jgConstraintJacobian_kPlus1, problem);
			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
				problem);
			solver.evaluateAugmentedLagrangianGradients(&problem.objectiveGradient_kPlus1[0],
				problem.hConstraintFunctions_kPlus1, problem.gConstraintFunctions_kPlus1,
				problem.jhConstraintJacobian_kPlus1, problem.jgConstraintJacobian_kPlus1,
				problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_kPlus1,
				problem.augmentedLagrangianGradientX_kPlus1, problem.augmentedLagrangianGradientZ_kPlus1,
				problem.augmentedLagrangianGradientMu_kPlus1, problem.augmentedLagrangianGradientLambda_kPlus1,
				problem);
			solver.updateCorrectionVectors(problem.xDesignVariables_k, problem.xDesignVariables_kPlus1,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientX_kPlus1,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.sVectors, problem.yVectors,
				problem);
			solver.setVariablesForNextIteration(
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
			++problem.kIteration;
			++problem.kIterationSinceRestart;
			solver.identifyActiveInactiveVariableSet(problem.xDesignVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k,
				problem.inactiveVariableSet_k, problem);
			solver.estimateActiveInactiveConstraintSet(problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
				problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
				problem);
			solver.evaluateBMatrix(problem.kIterationSinceRestart,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.BMatrix_k, problem);
			solver.evaluateBVector(problem.kIterationSinceRestart,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.augmentedLagrangianGradientX_k, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.bVector_k, problem);

			Assert::AreEqual(-0.11581251098102197, problem.bVector_k[0]);
			Assert::AreEqual(-0.85310462382124186, problem.bVector_k[1]);

		}

		TEST_METHOD(evaluateBMatrix)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			problem.phPenalties_k[0] = 1.0;
			problem.pgPenalties_k[0] = 1.0;
			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
				problem);
			solver.identifyActiveInactiveVariableSet(problem.xDesignVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k,
				problem.inactiveVariableSet_k, problem);
			solver.estimateActiveInactiveConstraintSet(problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
				problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
				problem);
			solver.evaluateBMatrix(problem.kIterationSinceRestart,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.BMatrix_k, problem);

			Assert::AreEqual(200.0, problem.BMatrix_k[0][0]);
			//Assert::AreEqual(-100.0, problem.BMatrix_k[0][1]);
			//Assert::AreEqual(-100.0, problem.BMatrix_k[1][0]);
			//Assert::AreEqual(50.0, problem.BMatrix_k[1][1]);

			solver.evaluateInactiveDesignVariableSearchDirection(problem.kIterationSinceRestart,
				problem.augmentedLagrangianGradientX_k, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);
			solver.ensureInactiveSearchDirectionThatOfDescent(problem.dxSearchDirection_k,
				problem.augmentedLagrangianGradientX_k, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem);
			solver.evaluatePhiPrime0LineSearchFunctionGradient(
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k,
				problem.augmentedLagrangianGradientMu_k, problem.dmuSearchDirection_k,
				problem.augmentedLagrangianGradientLambda_k, problem.dlambdaSearchDirection_k,
				problem.phiPrime0_k, problem);
			solver.initializeStepSize(problem.alphaStepSize_k,
				problem.zSlackVariables_k, problem.dzSearchDirection_k,
				problem.lambdaLagrangeMultipliers_k, problem.dlambdaSearchDirection_k,
				problem);
			solver.identifyStepSize(problem.alphaStepSize_k, problem.kIterationSinceRestart,
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
			solver.evaluateObjectiveGradient(&problem.xDesignVariables_kPlus1[0],
				&problem.objectiveGradient_kPlus1[0], problem);
			solver.evaluateAuxiliaryJacobians(&problem.xDesignVariables_kPlus1[0],
				problem.jhConstraintJacobian_kPlus1, problem.jgConstraintJacobian_kPlus1, problem);
			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
				problem);
			solver.updateCorrectionVectors(problem.xDesignVariables_k, problem.xDesignVariables_kPlus1,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientX_kPlus1,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.sVectors, problem.yVectors,
				problem);
			solver.setVariablesForNextIteration(
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
			++problem.kIteration;
			++problem.kIterationSinceRestart;
			solver.identifyActiveInactiveVariableSet(problem.xDesignVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k,
				problem.inactiveVariableSet_k, problem);
			solver.estimateActiveInactiveConstraintSet(problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
				problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
				problem);
			solver.evaluateBMatrix(problem.kIterationSinceRestart,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.BMatrix_k, problem);

			Assert::AreEqual(0.66013347224091645, problem.BMatrix_k[0][0]);
			//Assert::AreEqual(-0.44649820676022106, problem.BMatrix_k[0][1]);
			//Assert::AreEqual(-0.44649820676022106, problem.BMatrix_k[1][0]);
			//Assert::AreEqual(3.5990504785383681, problem.BMatrix_k[1][1]);

		}

		TEST_METHOD(evaluateCVector)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;

			problem.lagrangianGradientQP = { 1.0, 2.0, 3.0, 4.0 };
			problem.lagrangianGradient_k = { 5.0, 6.0, 7.0, 8.0 };
			problem.dxSearchDirection_k = { 9.0, 10.0, 11.0, 12.0 };
			problem.lambdaLagrangeMultipliers_k = { 13.0 };
			problem.dzSearchDirection_k = { 14.0 };
			problem.hConstraintFunctions_k = { 15.0 };
			problem.dmuSearchDirection_k = { 16.0 };
			problem.gConstraintFunctions_k = { 17.0 };
			problem.zSlackVariables_k = { 18.0 };
			problem.dlambdaSearchDirection_k = { 19.0 };

			// Evaluate cVector.
			solver.evaluateCVector(problem.lagrangianGradientQP,
				problem.lagrangianGradient_k, problem.dxSearchDirection_k,
				problem.lambdaLagrangeMultipliers_k, problem.dzSearchDirection_k,
				problem.hConstraintFunctions_k, problem.dmuSearchDirection_k,
				problem.gConstraintFunctions_k, problem.zSlackVariables_k,
				problem.dlambdaSearchDirection_k, problem.cVector_k, problem);

			Assert::AreEqual(-1310.0, problem.cVector_k[0]);

		}

		TEST_METHOD(evaluateCMatrix)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;

			problem.dxSearchDirection_k = { 9.0, 10.0, 11.0, 12.0 };
			problem.dzSearchDirection_k = { 14.0 };
			problem.hConstraintFunctions_k = { 15.0 };
			problem.gConstraintFunctions_k = { 17.0 };
			problem.zSlackVariables_k = { 18.0 };
			problem.jhConstraintJacobian_k = { { 19.0, 20.0, 21.0, 22.0 } };
			problem.jgConstraintJacobian_k = { { 23.0, 24.0, 25.0, 26.0 } };

			// Evaluate cMatrix.
			solver.evaluateCMatrix(problem.dxSearchDirection_k, problem.dzSearchDirection_k,
				problem.jhConstraintJacobian_k, problem.hConstraintFunctions_k,
				problem.jgConstraintJacobian_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k,
				problem.CMatrix_k, problem);

			Assert::AreEqual(12990.0, problem.CMatrix_k[0]);
			Assert::AreEqual(36680.0, problem.CMatrix_k[1]);

		}

		//TEST_METHOD(updatePenaltyParameterVectors)
		//{

		//integer n = 4;
		//LASO::HS071 hs071;
		//vector x = { 1.0, 5.0, 5.0, 1.0 };
		//vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//integer m = 2;
		//LASO::Options options;
		//LASO::ProblemAL problem(n, hs071, x, lb, ub, m, options);
		//LASO::SolverAL solver;

		//problem.dxSearchDirection_k = { 9.0, 10.0, 11.0, 12.0 };
		//problem.dzSearchDirection_k = { 14.0 };
		//problem.hConstraintFunctions_k = { 15.0 };
		//problem.gConstraintFunctions_k = { 17.0 };
		//problem.zSlackVariables_k = { 18.0 };
		//problem.jhConstraintJacobian_k = { { 19.0, 20.0, 21.0, 22.0 } };
		//problem.jgConstraintJacobian_k = { { 23.0, 24.0, 25.0, 26.0 } };

		//// Evaluate cMatrix.
		//solver.evaluateCMatrix(problem.dxSearchDirection_k, problem.dzSearchDirection_k,
		//	problem.jhConstraintJacobian_k, problem.hConstraintFunctions_k,
		//	problem.jgConstraintJacobian_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k,
		//	problem.CMatrix_k, problem);

		//Assert::AreEqual(12990.0, problem.CMatrix_k[0]);
		//Assert::AreEqual(36680.0, problem.CMatrix_k[1]);

		//}

		TEST_METHOD(ensureInactiveSearchDirectionThatOfDescent)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			//solver.evaluateReformattedConstraints(problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k,
			//	problem.rConstraintFunctions_k, problem);
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			problem.muLagrangeMultipliers_k[0] = 2.0;
			problem.lambdaLagrangeMultipliers_k[0] = 1.0;
			problem.phPenalties_k[0] = 1.0;
			problem.pgPenalties_k[0] = 1.0;
			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
				problem);
			solver.identifyActiveInactiveVariableSet(problem.xDesignVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k,
				problem.inactiveVariableSet_k, problem);
			solver.estimateActiveInactiveConstraintSet(problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
				problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
				problem);
			solver.evaluateInactiveDesignVariableSearchDirection(problem.kIterationSinceRestart,
				problem.augmentedLagrangianGradientX_k, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);
			problem.kIterationSinceRestart = 1;
			solver.ensureInactiveSearchDirectionThatOfDescent(problem.dxSearchDirection_k,
				problem.augmentedLagrangianGradientX_k, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem);

			Assert::AreEqual(193.04144632694815, problem.directionMagnitude);
			Assert::AreEqual(193.04144632694815, problem.gradientMagnitude);
			Assert::AreEqual(-37265.000000000000, problem.directionDotGradient);
			Assert::AreEqual((integer)0, (integer)problem.correctionHistory.size());
			Assert::AreEqual((integer)1, problem.kIterationSinceRestart);
			Assert::AreEqual(-136.0, problem.dxSearchDirection_k[1]);
			Assert::AreEqual(-137.0, problem.dxSearchDirection_k[2]);

			problem.kIterationSinceRestart = 1;
			problem.correctionHistory.push_back(0);
			problem.dxSearchDirection_k[1] = problem.dxSearchDirection_k[1] / 10000.0;
			problem.dxSearchDirection_k[2] = problem.dxSearchDirection_k[2] / 10000.0;
			solver.ensureInactiveSearchDirectionThatOfDescent(problem.dxSearchDirection_k,
				problem.augmentedLagrangianGradientX_k, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem);

			Assert::AreEqual((integer)0, (integer)problem.correctionHistory.size());
			Assert::AreEqual((integer)0, problem.kIterationSinceRestart);
			Assert::AreEqual(-136.0, problem.dxSearchDirection_k[1]);
			Assert::AreEqual(-137.0, problem.dxSearchDirection_k[2]);

			problem.kIterationSinceRestart = 1;
			problem.correctionHistory.push_back(0);
			problem.dxSearchDirection_k[1] = problem.dxSearchDirection_k[1] * 100000.0;
			problem.dxSearchDirection_k[2] = problem.dxSearchDirection_k[2] * 100000.0;
			solver.ensureInactiveSearchDirectionThatOfDescent(problem.dxSearchDirection_k,
				problem.augmentedLagrangianGradientX_k, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem);

			Assert::AreEqual((integer)0, (integer)problem.correctionHistory.size());
			Assert::AreEqual((integer)0, problem.kIterationSinceRestart);
			Assert::AreEqual(-136.0, problem.dxSearchDirection_k[1]);
			Assert::AreEqual(-137.0, problem.dxSearchDirection_k[2]);

			problem.kIterationSinceRestart = 1;
			problem.correctionHistory.push_back(0);
			problem.dxSearchDirection_k[1] = -problem.dxSearchDirection_k[1];
			problem.dxSearchDirection_k[2] = -problem.dxSearchDirection_k[2];
			solver.ensureInactiveSearchDirectionThatOfDescent(problem.dxSearchDirection_k,
				problem.augmentedLagrangianGradientX_k, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem);

			Assert::AreEqual((integer)0, (integer)problem.correctionHistory.size());
			Assert::AreEqual((integer)0, problem.kIterationSinceRestart);
			Assert::AreEqual(-136.0, problem.dxSearchDirection_k[1]);
			Assert::AreEqual(-137.0, problem.dxSearchDirection_k[2]);

		}

		TEST_METHOD(initializeStepSize)
		{
			//LASO::SolverAL solver;


			//solver.initializeStepSize(problem.alphaStepSize_k,
			//	problem.zSlackVariables_k, problem.dzSearchDirection_k,
			//	problem.lambdaLagrangeMultipliers_k, problem.dlambdaSearchDirection_k,
			//	problem);
		}

		TEST_METHOD(returnLineSearchFunction)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			boolean evaluateFunctions = true;
			vector fLineSearchFunction = { 0.0 };
			doublereal stepSize = 0.0;
			problem.phPenalties_k[0] = 1.0;
			problem.pgPenalties_k[0] = 1.0;
			fLineSearchFunction[0] = solver.returnLineSearchFunction(stepSize,
				problem.xDesignVariables_k, problem.zSlackVariables_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.dxSearchDirection_k, problem.dzSearchDirection_k,
				problem.dmuSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.phPenalties_k, problem.pgPenalties_k,
				&problem.xDesignVariables_kPlus1[0], problem.zSlackVariables_kPlus1,
				problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
				evaluateFunctions, &problem.fObjectiveFunction_kPlus1[0],
				problem.hConstraintFunctions_kPlus1, problem.gConstraintFunctions_kPlus1,
				problem);
			Assert::AreEqual(88.0, fLineSearchFunction[0]);

			problem.muLagrangeMultipliers_k[0] = 1.0;
			problem.lambdaLagrangeMultipliers_k[0] = 1.0;
			fLineSearchFunction[0] = solver.returnLineSearchFunction(stepSize,
				problem.xDesignVariables_k, problem.zSlackVariables_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.dxSearchDirection_k, problem.dzSearchDirection_k,
				problem.dmuSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.phPenalties_k, problem.pgPenalties_k,
				&problem.xDesignVariables_kPlus1[0], problem.zSlackVariables_kPlus1,
				problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
				evaluateFunctions, &problem.fObjectiveFunction_kPlus1[0],
				problem.hConstraintFunctions_kPlus1, problem.gConstraintFunctions_kPlus1,
				problem);
			Assert::AreEqual(100.0, fLineSearchFunction[0]);

			stepSize = 1.0;
			problem.dxSearchDirection_k[0] = 1.0;
			problem.dxSearchDirection_k[1] = 1.0;
			problem.dxSearchDirection_k[2] = 1.0;
			problem.dxSearchDirection_k[3] = 1.0;
			problem.dzSearchDirection_k[0] = 1.0;
			problem.dmuSearchDirection_k[0] = 1.0;
			problem.dlambdaSearchDirection_k[0] = 1.0;
			fLineSearchFunction[0] = solver.returnLineSearchFunction(stepSize,
				problem.xDesignVariables_k, problem.zSlackVariables_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.dxSearchDirection_k, problem.dzSearchDirection_k,
				problem.dmuSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.phPenalties_k, problem.pgPenalties_k,
				&problem.xDesignVariables_kPlus1[0], problem.zSlackVariables_kPlus1,
				problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
				evaluateFunctions, &problem.fObjectiveFunction_kPlus1[0],
				problem.hConstraintFunctions_kPlus1, problem.gConstraintFunctions_kPlus1,
				problem);
			Assert::AreEqual(2.0, problem.xDesignVariables_kPlus1[0]);
			Assert::AreEqual(5.0, problem.xDesignVariables_kPlus1[1]);
			Assert::AreEqual(5.0, problem.xDesignVariables_kPlus1[2]);
			Assert::AreEqual(2.0, problem.xDesignVariables_kPlus1[3]);
			Assert::AreEqual(1.0, problem.zSlackVariables_kPlus1[0]);
			Assert::AreEqual(2.0, problem.muLagrangeMultipliers_kPlus1[0]);
			Assert::AreEqual(2.0, problem.lambdaLagrangeMultipliers_kPlus1[0]);

		}

		TEST_METHOD(returnAugmentedLagrangianFunction)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			boolean evaluateFunctions = true;
			vector fAugmentedLagrangian = { 0.0 };
			problem.phPenalties_k[0] = 1.0;
			problem.pgPenalties_k[0] = 1.0;
			fAugmentedLagrangian[0] = solver.returnAugmentedLagrangianFunction(
				problem.xDesignVariables_k, problem.zSlackVariables_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.phPenalties_k, problem.pgPenalties_k,
				evaluateFunctions, problem.fObjectiveFunction_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem);
			Assert::AreEqual(88.0, fAugmentedLagrangian[0]);

			problem.muLagrangeMultipliers_k[0] = 1.0;
			problem.lambdaLagrangeMultipliers_k[0] = 1.0;
			fAugmentedLagrangian[0] = solver.returnAugmentedLagrangianFunction(
				problem.xDesignVariables_k, problem.zSlackVariables_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.phPenalties_k, problem.pgPenalties_k,
				evaluateFunctions, problem.fObjectiveFunction_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem);
			Assert::AreEqual(100.0, fAugmentedLagrangian[0]);

		}

		TEST_METHOD(returnPhiPrime0LineSearchFunctionGradient)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			problem.phPenalties_k[0] = 1.0;
			problem.pgPenalties_k[0] = 1.0;
			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			//solver.evaluateReformattedConstraints(problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem.zSlackVariables_k,
			//	problem.rConstraintFunctions_k, problem);
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			problem.muLagrangeMultipliers_k[0] = 2.0;
			problem.lambdaLagrangeMultipliers_k[0] = 1.0;
			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
				problem);

			problem.dxSearchDirection_k[0] = 1.0;
			problem.dxSearchDirection_k[1] = 1.0;
			problem.dxSearchDirection_k[2] = 1.0;
			problem.dxSearchDirection_k[3] = 1.0;

			problem.dzSearchDirection_k[0] = 1.0;

			problem.dmuSearchDirection_k[0] = 1.0;

			problem.dlambdaSearchDirection_k[0] = 1.0;

			solver.evaluatePhiPrime0LineSearchFunctionGradient(
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k,
				problem.augmentedLagrangianGradientMu_k, problem.dmuSearchDirection_k,
				problem.augmentedLagrangianGradientLambda_k, problem.dlambdaSearchDirection_k,
				problem.phiPrime0_k, problem);

			Assert::AreEqual(315.0, problem.phiPrime0_k[0]);

		}

		TEST_METHOD(testSufficientDecreaseCondition)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			doublereal alpha = 1.0;
			doublereal phi = 5.0;
			doublereal phi0 = 6.0;
			problem.phiPrime0_k[0] = -10000.0;
			boolean result = solver.testSufficientDecreaseCondition(phi, phi0, alpha, problem.phiPrime0_k, problem);
			Assert::AreEqual(true, result);

			problem.phiPrime0_k[0] = -10001.0;
			result = solver.testSufficientDecreaseCondition(phi, phi0, alpha, problem.phiPrime0_k, problem);
			Assert::AreEqual(false, result);

			problem.phiPrime0_k[0] = -10000.0;
			phi0 = 5.999;
			result = solver.testSufficientDecreaseCondition(phi, phi0, alpha, problem.phiPrime0_k, problem);
			Assert::AreEqual(false, result);

		}

		//TEST_METHOD(testCurvatureCondition)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	LASO::ProblemAL problem(n, hs071, x, lb, ub, m, options);
		//	LASO::SolverAL solver;
		//	doublereal alpha = 1.0;
		//	doublereal phi = 5.0;
		//	doublereal phi0 = 6.0;
		//	doublereal phiPrime0 = -10000.0;
		//	boolean result = solver.testCurvatureCondition(phi, phi0, alpha, phiPrime0, problem);
		//	Assert::AreEqual(true, result);

		//	phiPrime0 = -10001.0;
		//	result = solver.testCurvatureCondition(phi, phi0, alpha, phiPrime0, problem);
		//	Assert::AreEqual(false, result);

		//	phiPrime0 = -10000.0;
		//	phi0 = 5.999;
		//	result = solver.testCurvatureCondition(phi, phi0, alpha, phiPrime0, problem);
		//	Assert::AreEqual(false, result);

		//}

		TEST_METHOD(setVariablesForNextIteration)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
			//solver.evaluateReformattedConstraints(problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
			//	problem.zSlackVariables_k, problem.rConstraintFunctions_k, problem);
			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
			problem.muLagrangeMultipliers_k[0] = 2.0;
			problem.lambdaLagrangeMultipliers_k[0] = 1.0;
			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
				problem);
			solver.identifyActiveInactiveVariableSet(problem.xDesignVariables_k,
				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k,
				problem.inactiveVariableSet_k, problem);
			solver.estimateActiveInactiveConstraintSet(problem.zSlackVariables_k,
				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
				problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
				problem);
			solver.evaluateInactiveDesignVariableSearchDirection(problem.kIterationSinceRestart,
				problem.augmentedLagrangianGradientX_k, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);

			problem.zSlackVariables_k[0] = 1.0;
			problem.fObjectiveFunction_k[0] = 1.0;
			problem.gConstraintFunctions_k[0] = 1.0;
			problem.augmentedLagrangianGradientLambda_k[0] = 1.0;
			problem.augmentedLagrangianGradientMu_k[0] = 1.0;
			problem.lagrangianGradient_k[0] = 1.0;
			problem.lagrangianGradient_k[1] = 1.0;
			problem.lagrangianGradient_k[2] = 1.0;
			problem.lagrangianGradient_k[3] = 1.0;
			solver.setVariablesForNextIteration(
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

			Assert::AreEqual(0.0, problem.xDesignVariables_k[0]);
			Assert::AreEqual(0.0, problem.xDesignVariables_k[1]);
			Assert::AreEqual(0.0, problem.xDesignVariables_k[2]);
			Assert::AreEqual(0.0, problem.xDesignVariables_k[3]);

			Assert::AreEqual(0.0, problem.zSlackVariables_k[0]);

			Assert::AreEqual(0.0, problem.muLagrangeMultipliers_k[0]);

			Assert::AreEqual(0.0, problem.lambdaLagrangeMultipliers_k[0]);

			Assert::AreEqual(0.0, problem.fObjectiveFunction_k[0]);

			Assert::AreEqual(0.0, problem.hConstraintFunctions_k[0]);

			Assert::AreEqual(0.0, problem.gConstraintFunctions_k[0]);

			Assert::AreEqual(0.0, problem.objectiveGradient_k[0]);
			Assert::AreEqual(0.0, problem.objectiveGradient_k[1]);
			Assert::AreEqual(0.0, problem.objectiveGradient_k[2]);
			Assert::AreEqual(0.0, problem.objectiveGradient_k[3]);

			Assert::AreEqual(0.0, problem.jhConstraintJacobian_k[0][0]);
			Assert::AreEqual(0.0, problem.jhConstraintJacobian_k[0][1]);
			Assert::AreEqual(0.0, problem.jhConstraintJacobian_k[0][2]);
			Assert::AreEqual(0.0, problem.jhConstraintJacobian_k[0][3]);

			Assert::AreEqual(0.0, problem.jgConstraintJacobian_k[0][0]);
			Assert::AreEqual(0.0, problem.jgConstraintJacobian_k[0][1]);
			Assert::AreEqual(0.0, problem.jgConstraintJacobian_k[0][2]);
			Assert::AreEqual(0.0, problem.jgConstraintJacobian_k[0][3]);

			Assert::AreEqual(0.0, problem.augmentedLagrangianGradientX_k[0]);
			Assert::AreEqual(0.0, problem.augmentedLagrangianGradientX_k[1]);
			Assert::AreEqual(0.0, problem.augmentedLagrangianGradientX_k[2]);
			Assert::AreEqual(0.0, problem.augmentedLagrangianGradientX_k[3]);

			Assert::AreEqual(0.0, problem.augmentedLagrangianGradientZ_k[0]);

			Assert::AreEqual(0.0, problem.augmentedLagrangianGradientMu_k[0]);

			Assert::AreEqual(0.0, problem.augmentedLagrangianGradientLambda_k[0]);

			Assert::AreEqual(0.0, problem.lagrangianGradient_k[0]);
			Assert::AreEqual(0.0, problem.lagrangianGradient_k[1]);
			Assert::AreEqual(0.0, problem.lagrangianGradient_k[2]);
			Assert::AreEqual(0.0, problem.lagrangianGradient_k[3]);

		}

		TEST_METHOD(evaluateDesignVariableSearchDirection)
		{

			integer n = 2;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0 };
			vector lb = { -INFINITY, 1.5 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, &gObj[0]);

			// Test dxSearchDirection set to steepest descent direction on first iteration.
			LASO::SolverAL augmentedLagrangian;
			augmentedLagrangian.evaluateInactiveDesignVariableSearchDirection(problem.kIteration,
				gObj, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);
			Assert::AreEqual(2406.0, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(600.0, problem.dxSearchDirection_k[1]);

			// Test dxSearchDirection calculated correctly on second iteration.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			augmentedLagrangian.evaluateInactiveDesignVariableSearchDirection(problem.kIteration,
				gObj, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);
			Assert::AreEqual(2.0684411172726542, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(-0.20066555740432615, problem.dxSearchDirection_k[1]);

			// Test dxSearchDirection calculated correctly on second iteration first variable inactive.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			problem.inactiveVariableSet_k[0] = 0;
			problem.nINoInactiveVariables_k = 1;
			problem.dxSearchDirection_k[0] = 0.0;
			problem.dxSearchDirection_k[1] = 0.0;
			augmentedLagrangian.evaluateInactiveDesignVariableSearchDirection(problem.kIteration,
				gObj, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);
			Assert::AreEqual(2.0016638935108153, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(0.0, problem.dxSearchDirection_k[1]);

			// Test dxSearchDirection calculated correctly on third iteration using one correction.
			problem.kIteration = 2;
			problem.options.noCorrections = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.correctionHistory.push_back(1);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			problem.sVectors[1][0] = 0.0;
			problem.sVectors[1][1] = 0.8;
			problem.yVectors[1][0] = 0.0;
			problem.yVectors[1][1] = 160.0;
			problem.inactiveVariableSet_k[0] = 0;
			problem.inactiveVariableSet_k[1] = 1;
			problem.nINoInactiveVariables_k = 2;
			problem.dxSearchDirection_k[0] = 0.0;
			problem.dxSearchDirection_k[1] = 0.0;
			augmentedLagrangian.evaluateInactiveDesignVariableSearchDirection(problem.kIteration,
				gObj, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);
			Assert::AreEqual(12.030000000000001, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(3.0, problem.dxSearchDirection_k[1]);

			// Test dxSearchDirection calculated correctly on third iteration using two corrections.
			problem.kIteration = 2;
			problem.options.noCorrections = 2;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.correctionHistory.push_back(1);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			problem.sVectors[1][0] = 0.0;
			problem.sVectors[1][1] = 0.8;
			problem.yVectors[1][0] = 0.0;
			problem.yVectors[1][1] = 160.0;
			problem.inactiveVariableSet_k[0] = 0;
			problem.inactiveVariableSet_k[1] = 1;
			problem.nINoInactiveVariables_k = 2;
			problem.dxSearchDirection_k[0] = 0.0;
			problem.dxSearchDirection_k[1] = 0.0;
			augmentedLagrangian.evaluateInactiveDesignVariableSearchDirection(problem.kIteration,
				gObj, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);
			Assert::AreEqual(3.3338861188091951, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(3.0000000000000009, problem.dxSearchDirection_k[1]);

			// Test dxSearchDirection calculated correctly on fourth iteration using three corrections.
			problem.kIteration = 3;
			problem.options.noCorrections = 3;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.correctionHistory.push_back(1);
			problem.correctionHistory.push_back(2);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			problem.sVectors[1][0] = 0.0;
			problem.sVectors[1][1] = 0.8;
			problem.yVectors[1][0] = 0.0;
			problem.yVectors[1][1] = 160.0;
			problem.sVectors[2][0] = 0.0;
			problem.sVectors[2][1] = -0.3;
			problem.yVectors[2][0] = 0.0;
			problem.yVectors[2][1] = -60.0;
			problem.inactiveVariableSet_k[0] = 0;
			problem.inactiveVariableSet_k[1] = 1;
			problem.nINoInactiveVariables_k = 2;
			problem.dxSearchDirection_k[0] = 0.0;
			problem.dxSearchDirection_k[1] = 0.0;
			augmentedLagrangian.evaluateInactiveDesignVariableSearchDirection(problem.kIteration,
				gObj, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);
			Assert::AreEqual(3.3338861188091951, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(2.9999999999999996, problem.dxSearchDirection_k[1]);

			// Test dxSearchDirection calculated correctly on fourth iteration using two corrections.
			problem.kIteration = 3;
			problem.options.noCorrections = 2;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.correctionHistory.push_back(1);
			problem.correctionHistory.push_back(2);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			problem.sVectors[1][0] = 0.0;
			problem.sVectors[1][1] = 0.8;
			problem.yVectors[1][0] = 0.0;
			problem.yVectors[1][1] = 160.0;
			problem.sVectors[2][0] = 0.0;
			problem.sVectors[2][1] = -0.3;
			problem.yVectors[2][0] = 0.0;
			problem.yVectors[2][1] = -60.0;
			problem.inactiveVariableSet_k[0] = 0;
			problem.inactiveVariableSet_k[1] = 1;
			problem.nINoInactiveVariables_k = 2;
			problem.dxSearchDirection_k[0] = 0.0;
			problem.dxSearchDirection_k[1] = 0.0;
			augmentedLagrangian.evaluateInactiveDesignVariableSearchDirection(problem.kIteration,
				gObj, problem.sVectors, problem.yVectors,
				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
				problem.correctionHistory, problem.options.noCorrections,
				problem.dxSearchDirection_k, problem);
			Assert::AreEqual(12.030000000000001, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(3.0, problem.dxSearchDirection_k[1]);

		}

		// Set dxSearchDirection = -gradient.
		TEST_METHOD(setDirectionComponentsToNegOfAVector)
		{

			integer n = 4;
			LASO::HS071 hs071;
			vector x = { 1.0, 5.0, 5.0, 1.0 };
			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL augmentedLagrangian;
			augmentedLagrangian.evaluateObjectiveGradient(problem.xDesignVariables_k,
				problem.objectiveGradient_k, problem);
			augmentedLagrangian.setSearchDirectionToNegAVector(problem.dxSearchDirection_k, problem.objectiveGradient_k, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, 1.0, problem);
			Assert::AreEqual(-12.0, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(-1.0, problem.dxSearchDirection_k[1]);
			Assert::AreEqual(-2.0, problem.dxSearchDirection_k[2]);
			Assert::AreEqual(-11.0, problem.dxSearchDirection_k[3]);
			vectorinteger components = { 1, 2, 2, 3 };
			integer noComponents = 2;
			problem.objectiveGradient_k[1] = 0.0;
			problem.objectiveGradient_k[2] = 0.0;
			augmentedLagrangian.setSearchDirectionToNegAVector(problem.dxSearchDirection_k, problem.objectiveGradient_k, components, noComponents, 1.0, problem);
			Assert::AreEqual(-12.0, problem.dxSearchDirection_k[0]);
			Assert::AreEqual(0.0, problem.dxSearchDirection_k[1]);
			Assert::AreEqual(0.0, problem.dxSearchDirection_k[2]);
			Assert::AreEqual(-11.0, problem.dxSearchDirection_k[3]);

		}



		//// Return limit = 0 if iteration < noCorrections.  Return limit = iteration - noCorrections if iteration >= noCorrections.
		//TEST_METHOD(returnLimit)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	integer limit = 0;
		//	integer currentIteration = 0;
		//	LASO::SolverAL augmentedLagrangian;
		//	limit = augmentedLagrangian.returnLimit(currentIteration, problem.options.noCorrections, problem);
		//	Assert::AreEqual((integer)0, limit);
		//	currentIteration = 25;
		//	limit = augmentedLagrangian.returnLimit(currentIteration, problem.options.noCorrections, problem);
		//	Assert::AreEqual((integer)5, limit);

		//}



	};

}