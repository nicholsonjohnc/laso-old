#include "CppUnitTest.h"

#include "SolverAL.h"
#include "INCONSISTENT.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(INCONSISTENTTest)
	{

	public:

		TEST_METHOD(fObjEvaluation)
		{

			integer n = 1;
			LASO::INCONSISTENT inconsistent;
			vector x = { -20.0 };
			vector lb = { -INFINITY, -INFINITY, 4.0 };
			vector ub = { INFINITY, 1.0, INFINITY };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, inconsistent, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(20.0, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
		{

			integer n = 1;
			LASO::INCONSISTENT inconsistent;
			vector x = { -20.0 };
			vector lb = { -INFINITY, -INFINITY, 4.0 };
			vector ub = { INFINITY, 1.0, INFINITY };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, inconsistent, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(-1.0, problem.objectiveGradient_k[0]);

		}

		TEST_METHOD(fConEvaluation)
		{

			integer n = 1;
			LASO::INCONSISTENT inconsistent;
			vector x = { -20.0 };
			vector lb = { -INFINITY, -INFINITY, 4.0 };
			vector ub = { INFINITY, 1.0, INFINITY };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, inconsistent, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conFun(problem.xDesignVariables_k, problem.fConstraintFunctions_k);
			Assert::AreEqual(-20.0, problem.fConstraintFunctions_k[0]);
			Assert::AreEqual(400.0, problem.fConstraintFunctions_k[1]);

		}

		TEST_METHOD(jConEvaluation)
		{

			integer n = 1;
			LASO::INCONSISTENT inconsistent;
			vector x = { -20.0 };
			vector lb = { -INFINITY, -INFINITY, 4.0 };
			vector ub = { INFINITY, 1.0, INFINITY };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, inconsistent, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conJac(problem.xDesignVariables_k, problem.jConstraintJacobian_k);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[0 * m + 0]);
			Assert::AreEqual(-40.0, problem.jConstraintJacobian_k[0 * m + 1]);

		}

		TEST_METHOD(INCONSISTENT)
		{

			integer n = 1;
			LASO::INCONSISTENT inconsistent;
			vector x = { 1.0 };
			vector lb = { -INFINITY, -INFINITY, 4.0 };
			vector ub = { INFINITY, 1.0, INFINITY };
			integer m = 2;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			options.output = true;
			LASO::ProblemAL problem(n, inconsistent, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			problem.pgPenalties_k[0] = 0.1;
			problem.pgPenalties_k[1] = 0.1;
			solver.solve(problem);


			//integer n = 1;
			//LASO::INCONSISTENT inconsistent;
			//vector x = { -20.0 };
			//vector lb = { -INFINITY, 4.0 };
			//vector ub = { 1.0, INFINITY };
			//integer m = 1;
			//LASO::Options options;
			//vector fObj(1);
			//vector gObj(n);
			//vector fCon(m);
			//vector gCon(m * n);
			//options.output = true;
			//LASO::ProblemAL problem(n, inconsistent, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			//LASO::SolverAL solver;
			//problem.phPenalties_k[0] = 10.0;
			//solver.solve(problem);

		}

	};
}