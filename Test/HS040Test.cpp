#include "CppUnitTest.h"

#include "SolverAL.h"
#include "HS040.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(HS040Test)
	{

	public:

		TEST_METHOD(fObjEvaluation)
		{

			integer n = 4;
			LASO::HS040 hs040;
			vector x = { 1.0, 2.0, 3.0, 4.0 };
			vector lb = { -INFINITY, -INFINITY, -INFINITY, -INFINITY, 0.0, 0.0, 0.0 };
			vector ub = { INFINITY, INFINITY, INFINITY, INFINITY, 0.0, 0.0, 0.0 };
			integer m = 3;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs040, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(-24.0, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
		{

			integer n = 4;
			LASO::HS040 hs040;
			vector x = { 1.0, 2.0, 3.0, 4.0 };
			vector lb = { -INFINITY, -INFINITY, -INFINITY, -INFINITY, 0.0, 0.0, 0.0 };
			vector ub = { INFINITY, INFINITY, INFINITY, INFINITY, 0.0, 0.0, 0.0 };
			integer m = 3;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs040, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(-24.0, problem.objectiveGradient_k[0]);
			Assert::AreEqual(-12.0, problem.objectiveGradient_k[1]);
			Assert::AreEqual(-8.0, problem.objectiveGradient_k[2]);
			Assert::AreEqual(-6.0, problem.objectiveGradient_k[3]);

		}

		TEST_METHOD(fConEvaluation)
		{

			integer n = 4;
			LASO::HS040 hs040;
			vector x = { 1.0, 2.0, 3.0, 4.0 };
			vector lb = { -INFINITY, -INFINITY, -INFINITY, -INFINITY, 0.0, 0.0, 0.0 };
			vector ub = { INFINITY, INFINITY, INFINITY, INFINITY, 0.0, 0.0, 0.0 };
			integer m = 3;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs040, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conFun(problem.xDesignVariables_k, problem.fConstraintFunctions_k);
			Assert::AreEqual(4.0, problem.fConstraintFunctions_k[0]);
			Assert::AreEqual(1.0, problem.fConstraintFunctions_k[1]);
			Assert::AreEqual(14.0, problem.fConstraintFunctions_k[2]);

		}

		TEST_METHOD(jConEvaluation)
		{

			integer n = 4;
			LASO::HS040 hs040;
			vector x = { 1.0, 2.0, 3.0, 4.0 };
			vector lb = { -INFINITY, -INFINITY, -INFINITY, -INFINITY, 0.0, 0.0, 0.0 };
			vector ub = { INFINITY, INFINITY, INFINITY, INFINITY, 0.0, 0.0, 0.0 };
			integer m = 3;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs040, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conJac(problem.xDesignVariables_k, problem.jConstraintJacobian_k);
			Assert::AreEqual(3.0, problem.jConstraintJacobian_k[0 * m + 0]);
			Assert::AreEqual(4.0, problem.jConstraintJacobian_k[1 * m + 0]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[2 * m + 0]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[3 * m + 0]);

			Assert::AreEqual(8.0, problem.jConstraintJacobian_k[0 * m + 1]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[1 * m + 1]);
			Assert::AreEqual(-1.0, problem.jConstraintJacobian_k[2 * m + 1]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[3 * m + 1]);

			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[0 * m + 2]);
			Assert::AreEqual(-1.0, problem.jConstraintJacobian_k[1 * m + 2]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[2 * m + 2]);
			Assert::AreEqual(8.0, problem.jConstraintJacobian_k[3 * m + 2]);

		}

		TEST_METHOD(HS040)
		{

			integer n = 4;
			LASO::HS040 hs040;
			vector x = { 0.8, 0.8, 0.8, 0.8 };
			vector lb = { -INFINITY, -INFINITY, -INFINITY, -INFINITY, 0.0, 0.0, 0.0 };
			vector ub = { INFINITY, INFINITY, INFINITY, INFINITY, 0.0, 0.0, 0.0 };
			integer m = 3;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			//options.output = true;
			LASO::ProblemAL problem(n, hs040, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			problem.phPenalties_k[0] = 1000.0;
			problem.phPenalties_k[1] = 1000.0;
			problem.phPenalties_k[2] = 1000.0;
			//solver.solve(problem);

		}

	};
}