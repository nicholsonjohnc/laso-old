#include "CppUnitTest.h"

#include "SolverAL.h"
#include "HS006.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(HS006Test)
	{

	public:

		TEST_METHOD(fObjEvaluation)
		{

			integer n = 2;
			LASO::HS006 hs006;
			vector x = { -1.2, 1.0 };
			vector lb = { -INFINITY, -INFINITY, 0.0 };
			vector ub = { INFINITY, INFINITY, 0.0 };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs006, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(4.8400000000000007, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
		{

			integer n = 2;
			LASO::HS006 hs006;
			vector x = { -1.2, 1.0 };
			vector lb = { -INFINITY, -INFINITY, 0.0 };
			vector ub = { INFINITY, INFINITY, 0.0 };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs006, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(-4.4000000000000004, problem.objectiveGradient_k[0]);
			Assert::AreEqual(0.0, problem.objectiveGradient_k[1]);

		}

		TEST_METHOD(fConEvaluation)
		{

			integer n = 2;
			LASO::HS006 hs006;
			vector x = { -1.2, 1.0 };
			vector lb = { -INFINITY, -INFINITY, 0.0 };
			vector ub = { INFINITY, INFINITY, 0.0 };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs006, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conFun(problem.xDesignVariables_k, problem.fConstraintFunctions_k);
			Assert::AreEqual(-4.3999999999999995, problem.fConstraintFunctions_k[0]);

		}

		TEST_METHOD(jConEvaluation)
		{

			integer n = 2;
			LASO::HS006 hs006;
			vector x = { -1.2, 1.0 };
			vector lb = { -INFINITY, -INFINITY, 0.0 };
			vector ub = { INFINITY, INFINITY, 0.0 };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs006, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conJac(problem.xDesignVariables_k, problem.jConstraintJacobian_k);
			Assert::AreEqual(24.0, problem.jConstraintJacobian_k[0 * m + 0]);
			Assert::AreEqual(10.0, problem.jConstraintJacobian_k[1 * m + 0]);

		}

		TEST_METHOD(HS006)
		{

			integer n = 2;
			LASO::HS006 hs006;
			vector x = { -1.2, 1.0 };
			vector lb = { -INFINITY, -INFINITY, 0.0 };
			vector ub = { INFINITY, INFINITY, 0.0 };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			//options.output = true;
			LASO::ProblemAL problem(n, hs006, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			//problem.phPenalties_k[0] = 0.001;
			solver.solve(problem);

		}

	};
}