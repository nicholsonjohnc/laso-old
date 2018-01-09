#include "CppUnitTest.h"

#include "ProblemAL.h"
#include "HS071.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(ProblemALTest)
	{

	public:

		TEST_METHOD(nDesignVariablesInitialization)
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
			Assert::AreEqual(n, problem.nDesignVariables);

		}

		TEST_METHOD(optimizationFunctionsInitialization)
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
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(16.0, problem.fObjectiveFunction_k[0]);

			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(12.0, problem.objectiveGradient_k[0]);
			Assert::AreEqual(1.0, problem.objectiveGradient_k[1]);
			Assert::AreEqual(2.0, problem.objectiveGradient_k[2]);
			Assert::AreEqual(11.0, problem.objectiveGradient_k[3]);

			problem.functions.conFun(problem.xDesignVariables_k, problem.fConstraintFunctions_k);
			Assert::AreEqual(0.0, problem.fConstraintFunctions_k[0]);
			Assert::AreEqual(12.0, problem.fConstraintFunctions_k[1]);

			problem.functions.conJac(problem.xDesignVariables_k, problem.jConstraintJacobian_k);
			Assert::AreEqual(25.0, problem.jConstraintJacobian_k[0 * m + 0]);
			Assert::AreEqual(5.0, problem.jConstraintJacobian_k[1 * m + 0]);
			Assert::AreEqual(5.0, problem.jConstraintJacobian_k[2 * m + 0]);
			Assert::AreEqual(25.0, problem.jConstraintJacobian_k[3 * m + 0]);

			Assert::AreEqual(2.0, problem.jConstraintJacobian_k[0 * m + 1]);
			Assert::AreEqual(10.0, problem.jConstraintJacobian_k[1 * m + 1]);
			Assert::AreEqual(10.0, problem.jConstraintJacobian_k[2 * m + 1]);
			Assert::AreEqual(2.0, problem.jConstraintJacobian_k[3 * m + 1]);

		}

		TEST_METHOD(xDesignVariables_kInitialization)
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
			Assert::AreEqual(x[0], problem.xDesignVariables_k[0]);
			Assert::AreEqual(x[1], problem.xDesignVariables_k[1]);
			Assert::AreEqual(x[2], problem.xDesignVariables_k[2]);
			Assert::AreEqual(x[3], problem.xDesignVariables_k[3]);

		}

		TEST_METHOD(xDesignVariables_kPlus1Initialization)
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
			Assert::AreEqual((integer)4, (integer)problem.xDesignVariables_kPlus1.size());

		}

		TEST_METHOD(xDesignVariablesTrialInitialization)
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
			Assert::AreEqual((integer)4, (integer)problem.xDesignVariablesTrial.size());

		}

		TEST_METHOD(lbLowerBoundsInitialization)
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
			Assert::AreEqual(lb[0], problem.lbLowerBounds[0]);
			Assert::AreEqual(lb[1], problem.lbLowerBounds[1]);
			Assert::AreEqual(lb[2], problem.lbLowerBounds[2]);
			Assert::AreEqual(lb[3], problem.lbLowerBounds[3]);
			Assert::AreEqual(lb[4], problem.lbLowerBounds[4]);
			Assert::AreEqual(lb[5], problem.lbLowerBounds[5]);

		}

		TEST_METHOD(ubUpperBoundsInitialization)
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
			Assert::AreEqual(ub[0], problem.ubUpperBounds[0]);
			Assert::AreEqual(ub[1], problem.ubUpperBounds[1]);
			Assert::AreEqual(ub[2], problem.ubUpperBounds[2]);
			Assert::AreEqual(ub[3], problem.ubUpperBounds[3]);
			Assert::AreEqual(ub[4], problem.ubUpperBounds[4]);
			Assert::AreEqual(ub[5], problem.ubUpperBounds[5]);

		}

		TEST_METHOD(mConstraintsInitialization)
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
			Assert::AreEqual(m, problem.mConstraints);

		}

		TEST_METHOD(optimizationOptionsInitialization)
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
			options.majorFeasibilityTolerance = 5.0;
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			Assert::AreEqual(options.majorFeasibilityTolerance, problem.options.majorFeasibilityTolerance);
			Assert::AreEqual(options.majorOptimalityTolerance, problem.options.majorOptimalityTolerance);
			Assert::AreEqual(options.rhoFactor, problem.options.rhoFactor);

		}

		TEST_METHOD(kIterationInitialization)
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
			Assert::AreEqual((integer)0, problem.kIteration);

		}

		TEST_METHOD(kIterationSinceRestartInitialization)
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
			Assert::AreEqual((integer)0, problem.kIterationSinceRestart);

		}

		//TEST_METHOD(objectiveGradient_kInitialization)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	vector fObj(1);
		//	vector gObj(n);
		//	vector fCon(m);
		//	vector gCon(m * n);
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	Assert::AreEqual((integer)4, (integer)problem.objectiveGradient_k.size());

		//}

		TEST_METHOD(objectiveGradient_kPlus1Initialization)
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
			Assert::AreEqual((integer)4, (integer)problem.objectiveGradient_kPlus1.size());

		}

		TEST_METHOD(lagrangianGradient_kInitialization)
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
			Assert::AreEqual((integer)4, (integer)problem.lagrangianGradient_k.size());

		}

		TEST_METHOD(rLagrangianGradientQPInitialization)
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
			Assert::AreEqual((integer)4, (integer)problem.lagrangianGradientQP.size());

		}

		TEST_METHOD(lagrangianGradient_kPlus1Initialization)
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
			Assert::AreEqual((integer)4, (integer)problem.lagrangianGradient_kPlus1.size());

		}

		//TEST_METHOD(rConstraintFunctions_kInitialization)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//vector fObj(1);
		//vector gObj(n);
		//vector fCon(m);
		//vector gCon(m * n);
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	Assert::AreEqual((integer)2, (integer)problem.rConstraintFunctions_k.size());

		//}

		//TEST_METHOD(rConstraintFunctions_kPlus1Initialization)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	Assert::AreEqual((integer)2, (integer)problem.rConstraintFunctions_kPlus1.size());

		//}

		//TEST_METHOD(jrConstraintJacobian_kInitialization)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	// Test jrConstraintJacobian_k initialized to correct size.
		//	Assert::AreEqual((integer)2, (integer)problem.jrConstraintJacobian_k.size());
		//	Assert::AreEqual((integer)5, (integer)problem.jrConstraintJacobian_k[0].size());
		//	Assert::AreEqual((integer)5, (integer)problem.jrConstraintJacobian_k[1].size());
		//	// Test correct components of jrConstraintJacobian_k initialized to identity matrix.
		//	Assert::AreEqual(1.0, problem.jrConstraintJacobian_k[1][4]);

		//}

		//TEST_METHOD(jrConstraintJacobian_kPlus1Initialization)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	// Test jrConstraintJacobian_k initialized to correct size.
		//	Assert::AreEqual((integer)2, (integer)problem.jrConstraintJacobian_kPlus1.size());
		//	Assert::AreEqual((integer)5, (integer)problem.jrConstraintJacobian_kPlus1[0].size());
		//	Assert::AreEqual((integer)5, (integer)problem.jrConstraintJacobian_kPlus1[1].size());
		//	// Test correct components of jrConstraintJacobian_k initialized to identity matrix.
		//	Assert::AreEqual(1.0, problem.jrConstraintJacobian_kPlus1[1][4]);

		//}

		//TEST_METHOD(fObjectiveFunction_kInitialization)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	vector fObj(1);
		//	vector gObj(n);
		//	vector fCon(m);
		//	vector gCon(m * n);
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	Assert::AreEqual((integer)1, (integer)problem.fObjectiveFunction_k.size());

		//}

		TEST_METHOD(fObjectiveFunction_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.fObjectiveFunction_kPlus1.size());

		}

		TEST_METHOD(phiLineSearchFunctionInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.phiLineSearchFunction_k.size());

		}

		//TEST_METHOD(objectiveGradient_kInitialization)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	Assert::AreEqual((integer)4, (integer)problem.objectiveGradient_k.size());

		//}

		//TEST_METHOD(fConstraintFunctions_kInitialization)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	vector fObj(1);
		//	vector gObj(n);
		//	vector fCon(m);
		//	vector gCon(m * n);
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	Assert::AreEqual((integer)2, (integer)problem.fConstraintFunctions_k.size());

		//}

		//TEST_METHOD(jConstraintJacobian_kInitialization)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	vector fObj(1);
		//	vector gObj(n);
		//	vector fCon(m);
		//	vector gCon(m * n);
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	Assert::AreEqual((integer)8, (integer)problem.jConstraintJacobian_k.size());

		//}

		TEST_METHOD(nofObjEvaluationsInitialization)
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
			Assert::AreEqual((integer)0, problem.nofObjEvaluations);

		}

		TEST_METHOD(nofConEvaluationsInitialization)
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
			Assert::AreEqual((integer)0, problem.nofConEvaluations);

		}

		TEST_METHOD(nojConEvaluationsInitialization)
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
			Assert::AreEqual((integer)0, problem.nojConEvaluations);

		}

		TEST_METHOD(nogObjEvaluationsInitialization)
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
			Assert::AreEqual((integer)0, problem.nogObjEvaluations);

		}

		TEST_METHOD(noMinorIterationsInitialization)
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
			Assert::AreEqual((integer)0, problem.noMinorIterations);

		}

		TEST_METHOD(EqualityConstraintSetInitialization)
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
			Assert::AreEqual((integer)1, problem.EqualityConstraintSet[0]);

		}

		TEST_METHOD(InequalityConstraintSetUInitialization)
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
			Assert::AreEqual((integer)0, (integer)problem.InequalityConstraintSetU.size());

		}

		TEST_METHOD(InequalityConstraintSetLInitialization)
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
			Assert::AreEqual((integer)0, problem.InequalityConstraintSetL[0]);

		}

		TEST_METHOD(mENoEqualityConstraintsInitialization)
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
			Assert::AreEqual((integer)1, problem.mENoEqualityConstraints);

		}

		TEST_METHOD(mINoInequalityConstraintsUInitialization)
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
			Assert::AreEqual((integer)0, problem.mINoInequalityConstraintsU);

		}

		TEST_METHOD(mINoInequalityConstraintsLInitialization)
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
			Assert::AreEqual((integer)1, problem.mINoInequalityConstraintsL);

		}

		TEST_METHOD(zSlackVariables_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.zSlackVariables_k.size());

		}

		TEST_METHOD(zSlackVariables_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.zSlackVariables_kPlus1.size());

		}

		TEST_METHOD(activeConstraintSet_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.activeConstraintSet_k.size());

		}

		TEST_METHOD(inactiveConstraintSet_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.inactiveConstraintSet_k.size());

		}

		TEST_METHOD(inactiveVariableSet_kInitialization)
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
			Assert::AreEqual((integer)0, problem.inactiveVariableSet_k[0]);
			Assert::AreEqual((integer)1, problem.inactiveVariableSet_k[1]);
			Assert::AreEqual((integer)2, problem.inactiveVariableSet_k[2]);
			Assert::AreEqual((integer)3, problem.inactiveVariableSet_k[3]);

		}

		TEST_METHOD(nANoActiveConstraints_kInitialization)
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
			Assert::AreEqual((integer)0, problem.mANoActiveConstraints_k);

		}

		TEST_METHOD(noInactiveConstraints_kInitialization)
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
			Assert::AreEqual((integer)0, problem.mINoInactiveConstraints_k);

		}

		TEST_METHOD(nINoInactiveVariables_kInitialization)
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
			Assert::AreEqual((integer)4, problem.nINoInactiveVariables_k);

		}

		TEST_METHOD(hConstraintFunctions_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.hConstraintFunctions_k.size());

		}

		TEST_METHOD(hConstraintFunctions_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.hConstraintFunctions_kPlus1.size());

		}

		TEST_METHOD(gConstraintFunctions_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.gConstraintFunctions_k.size());

		}

		TEST_METHOD(gConstraintFunctions_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.gConstraintFunctions_kPlus1.size());

		}

		TEST_METHOD(jhConstraintJacobian_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.jhConstraintJacobian_k.size());
			Assert::AreEqual((integer)4, (integer)problem.jhConstraintJacobian_k[0].size());

		}

		TEST_METHOD(jhConstraintJacobian_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.jhConstraintJacobian_kPlus1.size());
			Assert::AreEqual((integer)4, (integer)problem.jhConstraintJacobian_kPlus1[0].size());

		}

		TEST_METHOD(jgConstraintJacobian_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.jgConstraintJacobian_k.size());
			Assert::AreEqual((integer)4, (integer)problem.jgConstraintJacobian_k[0].size());

		}

		TEST_METHOD(jgConstraintJacobian_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.jgConstraintJacobian_kPlus1.size());
			Assert::AreEqual((integer)4, (integer)problem.jgConstraintJacobian_kPlus1[0].size());

		}

		TEST_METHOD(constraintViolations_kInitialization)
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
			Assert::AreEqual((integer)2, (integer)problem.constraintViolations_k.size());

		}

		TEST_METHOD(maxConstraintViolation_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.maxConstraintViolation_k.size());
			Assert::AreEqual(0.0, problem.maxConstraintViolation_k[0]);

		}

		TEST_METHOD(optimalityViolations_kInitialization)
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
			Assert::AreEqual((integer)4, (integer)problem.optimalityViolations_k.size());

		}

		TEST_METHOD(maxOptimalityViolation_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.maxOptimalityViolation_k.size());

		}

		TEST_METHOD(maxMinorOptimalityViolation_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.maxMinorOptimalityViolation_k.size());

		}

		TEST_METHOD(lambdaLagrangeMultipliersL_kInitialization)
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
			Assert::AreEqual((integer)5, (integer)problem.lambdaLagrangeMultipliersL_k.size());

		}

		TEST_METHOD(lambdaLagrangeMultipliersU_kInitialization)
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
			Assert::AreEqual((integer)5, (integer)problem.lambdaLagrangeMultipliersU_k.size());

		}

		TEST_METHOD(lambdaLagrangeMultipliers_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.lambdaLagrangeMultipliers_k.size());

		}

		TEST_METHOD(lambdaLagrangeMultipliers_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.lambdaLagrangeMultipliers_kPlus1.size());

		}

		TEST_METHOD(lambdaLagrangeMultipliersQPInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.lambdaLagrangeMultipliersQP.size());

		}

		TEST_METHOD(muLagrangeMultipliers_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.muLagrangeMultipliers_k.size());

		}

		TEST_METHOD(muLagrangeMultipliers_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.muLagrangeMultipliers_kPlus1.size());

		}

		TEST_METHOD(muLagrangeMultipliersQPInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.muLagrangeMultipliersQP.size());

		}

		TEST_METHOD(dxSearchDirection_kInitialization)
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
			Assert::AreEqual((integer)4, (integer)problem.dxSearchDirection_k.size());

		}

		TEST_METHOD(dzSearchDirection_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.dzSearchDirection_k.size());

		}

		TEST_METHOD(dmuSearchDirection_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.dmuSearchDirection_k.size());

		}

		TEST_METHOD(dlambdaSearchDirection_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.dlambdaSearchDirection_k.size());

		}

		TEST_METHOD(phPenalties_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.phPenalties_k.size());
			Assert::AreEqual((integer)0.01, (integer)problem.phPenalties_k[0]);

		}

		//TEST_METHOD(phPenalties_kPlus1Initialization)
		//{

		//	integer n = 4;
		//	LASO::HS071 hs071;
		//	vector x = { 1.0, 5.0, 5.0, 1.0 };
		//	vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
		//	vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
		//	integer m = 2;
		//	LASO::Options options;
		//	LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	Assert::AreEqual((integer)2, (integer)problem.phPenalties_kPlus1.size());
		//	Assert::AreEqual((integer)1.0, (integer)problem.phPenalties_kPlus1[0]);
		//	Assert::AreEqual((integer)1.0, (integer)problem.phPenalties_kPlus1[1]);

		//}

		TEST_METHOD(pgPenalties_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.pgPenalties_k.size());
			Assert::AreEqual((integer)0.01, (integer)problem.pgPenalties_k[0]);

		}

		TEST_METHOD(augmentedLagrangianGradientX_kInitialization)
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
			Assert::AreEqual((integer)4, (integer)problem.augmentedLagrangianGradientX_k.size());

		}

		TEST_METHOD(augmentedLagrangianGradientXQPInitialization)
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
			Assert::AreEqual((integer)4, (integer)problem.augmentedLagrangianGradientXQP.size());

		}

		TEST_METHOD(augmentedLagrangianGradientX_kPlus1Initialization)
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
			Assert::AreEqual((integer)4, (integer)problem.augmentedLagrangianGradientX_kPlus1.size());

		}

		TEST_METHOD(augmentedLagrangianGradientZ_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.augmentedLagrangianGradientZ_k.size());

		}

		TEST_METHOD(augmentedLagrangianGradientZQPInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.augmentedLagrangianGradientZQP.size());

		}

		TEST_METHOD(augmentedLagrangianGradientZ_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.augmentedLagrangianGradientZ_kPlus1.size());

		}

		TEST_METHOD(augmentedLagrangianGradientMu_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.augmentedLagrangianGradientMu_k.size());

		}

		TEST_METHOD(augmentedLagrangianGradientMuQPInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.augmentedLagrangianGradientMuQP.size());

		}

		TEST_METHOD(augmentedLagrangianGradientMu_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.augmentedLagrangianGradientMu_kPlus1.size());

		}

		TEST_METHOD(augmentedLagrangianGradientLambda_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.augmentedLagrangianGradientLambda_k.size());

		}

		TEST_METHOD(augmentedLagrangianGradientLambdaQPInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.augmentedLagrangianGradientLambdaQP.size());

		}

		TEST_METHOD(augmentedLagrangianGradientLambda_kPlus1Initialization)
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
			Assert::AreEqual((integer)1, (integer)problem.augmentedLagrangianGradientLambda_kPlus1.size());

		}

		TEST_METHOD(phiPrime0_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.phiPrime0_k.size());

		}

		TEST_METHOD(rhoFactorInitialization)
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
			Assert::AreEqual(0.0001, problem.rhoFactor);

		}

		TEST_METHOD(betaFactorInitialization)
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
			Assert::AreEqual(0.9, problem.betaFactor);

		}

		TEST_METHOD(etaFactorInitialization)
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
			Assert::AreEqual((1.0 + sqrt(5.0)) / 2.0, problem.etaFactor);

		}

		TEST_METHOD(sigmaOneFactorInitialization)
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
			Assert::AreEqual(0.0002, problem.sigmaOneFactor);

		}

		TEST_METHOD(sigmaTwoFactorInitialization)
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
			Assert::AreEqual(sqrt(1000.0) * 1000.0, problem.sigmaTwoFactor);

		}

		TEST_METHOD(alphaStepSize_kInitialization)
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
			Assert::AreEqual(0.0, problem.alphaStepSize_k[0]);

		}

		TEST_METHOD(inverseHessianScaling_kInitialization)
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
			Assert::AreEqual(1.0, problem.inverseHessianScaling_k[0]);

		}

		TEST_METHOD(lineSearchProcedure_kInitialization)
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
			Assert::AreEqual(string("N/A"), problem.lineSearchProcedure_k);

		}

		TEST_METHOD(codeInitialization)
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
			Assert::AreEqual((integer)0, (integer)problem.code.size());

		}

		TEST_METHOD(qVectorInitialization)
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
			Assert::AreEqual((integer)5, (integer)problem.qVector.size());

		}

		TEST_METHOD(rVectorInitialization)
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
			Assert::AreEqual((integer)5, (integer)problem.rVector.size());

		}

		TEST_METHOD(correctionHistoryInitialization)
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
			Assert::AreEqual((integer)0, (integer)problem.correctionHistory.size());

		}

		TEST_METHOD(sVectorsInitialization)
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
			Assert::AreEqual((integer)20, (integer)problem.sVectors.size());
			Assert::AreEqual((integer)4, (integer)problem.sVectors[0].size());
			Assert::AreEqual((integer)4, (integer)problem.sVectors[19].size());

		}

		TEST_METHOD(yVectorsInitialization)
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
			Assert::AreEqual((integer)20, (integer)problem.yVectors.size());
			Assert::AreEqual((integer)4, (integer)problem.yVectors[0].size());
			Assert::AreEqual((integer)4, (integer)problem.yVectors[19].size());

		}

		TEST_METHOD(directionMagnitudeInitialization)
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
			Assert::AreEqual(0.0, problem.directionMagnitude);

		}

		TEST_METHOD(gradientMagnitudeInitialization)
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
			Assert::AreEqual(0.0, problem.gradientMagnitude);

		}

		TEST_METHOD(directionDotGradientInitialization)
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
			Assert::AreEqual(0.0, problem.directionDotGradient);

		}

		TEST_METHOD(bVector_kInitialization)
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
			Assert::AreEqual((integer)2, (integer)problem.bVector_k.size());

		}

		TEST_METHOD(BMatrix_kInitialization)
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
			Assert::AreEqual((integer)2, (integer)problem.BMatrix_k.size());
			Assert::AreEqual((integer)2, (integer)problem.BMatrix_k[0].size());
			Assert::AreEqual((integer)2, (integer)problem.BMatrix_k[1].size());

		}

		TEST_METHOD(lagrangeMultiplierSearchDirectionInitialization)
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
			Assert::AreEqual((integer)2, (integer)problem.lagrangeMultiplierSearchDirection.size());

		}

		TEST_METHOD(cVector_kInitialization)
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
			Assert::AreEqual((integer)1, (integer)problem.cVector_k.size());

		}

		TEST_METHOD(CVector_kInitialization)
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
			Assert::AreEqual((integer)2, (integer)problem.CMatrix_k.size());

		}

		TEST_METHOD(pSolutionInitialization)
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
			Assert::AreEqual((integer)3, (integer)problem.pSolution.size());

		}

	};

}