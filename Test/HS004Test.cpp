#include "CppUnitTest.h"

#include "SolverAL.h"
#include "HS004.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(HS004Test)
	{

	public:

		TEST_METHOD(fObjEvaluation)
		{

			integer n = 2;
			LASO::HS004 hs004;
			vector x = { 1.125, 0.125 };
			vector lb = { 1.0, 0.0 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs004, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(3.3235677083333330, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
		{

			integer n = 2;
			LASO::HS004 hs004;
			vector x = { 1.125, 0.125 };
			vector lb = { 1.0, 0.0 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs004, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(4.515625, problem.objectiveGradient_k[0]);
			Assert::AreEqual(1.0, problem.objectiveGradient_k[1]);

		}

		TEST_METHOD(HS004)
		{

			integer n = 2;
			LASO::HS004 hs004;
			vector x = { 1.125, 0.125 };
			vector lb = { 1.0, 0.0 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			//options.output = true;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs004, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;

			//solver.solve(problem);

		}

	};
}