#include "CppUnitTest.h"

#include "SolverAL.h"
#include "HS028.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(HS028Test)
	{

	public:

		TEST_METHOD(fObjEvaluation)
		{

			integer n = 3;
			LASO::HS028 hs028;
			vector x = { -4.0, 1.0, 1.0 };
			vector lb = { -INFINITY, -INFINITY, -INFINITY, 0.0 };
			vector ub = { INFINITY, INFINITY, INFINITY, 0.0 };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs028, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(13.0, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
		{

			integer n = 3;
			LASO::HS028 hs028;
			vector x = { 1.0, 2.0, 3.0 };
			vector lb = { -INFINITY, -INFINITY, -INFINITY, 0.0 };
			vector ub = { INFINITY, INFINITY, INFINITY, 0.0 };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs028, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(6.0, problem.objectiveGradient_k[0]);
			Assert::AreEqual(16.0, problem.objectiveGradient_k[1]);
			Assert::AreEqual(10.0, problem.objectiveGradient_k[2]);

		}

		TEST_METHOD(fConEvaluation)
		{

			integer n = 3;
			LASO::HS028 hs028;
			vector x = { 1.0, 2.0, 3.0 };
			vector lb = { -INFINITY, -INFINITY, -INFINITY, 0.0 };
			vector ub = { INFINITY, INFINITY, INFINITY, 0.0 };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs028, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conFun(problem.xDesignVariables_k, problem.fConstraintFunctions_k);
			Assert::AreEqual(13.0, problem.fConstraintFunctions_k[0]);

		}

		TEST_METHOD(jConEvaluation)
		{

			integer n = 3;
			LASO::HS028 hs028;
			vector x = { 1.0, 2.0, 3.0 };
			vector lb = { -INFINITY, -INFINITY, -INFINITY, 0.0 };
			vector ub = { INFINITY, INFINITY, INFINITY, 0.0 };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs028, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conJac(problem.xDesignVariables_k, problem.jConstraintJacobian_k);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[0 * m + 0]);
			Assert::AreEqual(2.0, problem.jConstraintJacobian_k[1 * m + 0]);
			Assert::AreEqual(3.0, problem.jConstraintJacobian_k[2 * m + 0]);

		}

		TEST_METHOD(HS028)
		{

			integer n = 3;
			LASO::HS028 hs028;
			vector x = { -4.0, 1.0, 1.0 };
			vector lb = { -INFINITY, -INFINITY, -INFINITY, 0.0 };
			vector ub = { INFINITY, INFINITY, INFINITY, 0.0 };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			//options.output = true;
			LASO::ProblemAL problem(n, hs028, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			problem.phPenalties_k[0] = 0.01;
			//solver.solve(problem);

		}

	};
}