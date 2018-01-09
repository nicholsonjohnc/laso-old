#include "CppUnitTest.h"

#include "SolverAL.h"
#include "HS002.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(HS002Test)
	{

	public:

		TEST_METHOD(fObjEvaluation)
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
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(909.0, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
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
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(-2406.0, problem.objectiveGradient_k[0]);
			Assert::AreEqual(-600.0, problem.objectiveGradient_k[1]);

		}

		TEST_METHOD(HS002)
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
			//options.output = true;
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			//solver.solve(problem);

		}

	};
}