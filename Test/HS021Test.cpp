#include "CppUnitTest.h"

#include "SolverAL.h"
#include "HS021.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(HS021Test)
	{

	public:

		TEST_METHOD(fObjEvaluation)
		{

			integer n = 2;
			LASO::HS021 hs021;
			vector x = { -1.0, -1.0 };
			vector lb = { 2.0, -50.0, 0.0 };
			vector ub = { 50.0, 50.0, INFINITY };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs021, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(-98.989999999999995, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
		{

			integer n = 2;
			LASO::HS021 hs021;
			vector x = { -1.0, -1.0 };
			vector lb = { 2.0, -50.0, 0.0 };
			vector ub = { 50.0, 50.0, INFINITY };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs021, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(-0.02, problem.objectiveGradient_k[0]);
			Assert::AreEqual(-2.0, problem.objectiveGradient_k[1]);

		}

		TEST_METHOD(fConEvaluation)
		{

			integer n = 2;
			LASO::HS021 hs021;
			vector x = { -1.0, -1.0 };
			vector lb = { 2.0, -50.0, 0.0 };
			vector ub = { 50.0, 50.0, INFINITY };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs021, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conFun(problem.xDesignVariables_k, problem.fConstraintFunctions_k);
			Assert::AreEqual(-19.0, problem.fConstraintFunctions_k[0]);

		}

		TEST_METHOD(jConEvaluation)
		{

			integer n = 2;
			LASO::HS021 hs021;
			vector x = { -1.0, -1.0 };
			vector lb = { 2.0, -50.0, 0.0 };
			vector ub = { 50.0, 50.0, INFINITY };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs021, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conJac(problem.xDesignVariables_k, problem.jConstraintJacobian_k);
			Assert::AreEqual(10.0, problem.jConstraintJacobian_k[0 * m + 0]);
			Assert::AreEqual(-1.0, problem.jConstraintJacobian_k[1 * m + 0]);

		}

		TEST_METHOD(HS021)
		{

			integer n = 2;
			LASO::HS021 hs021;
			vector x = { -1.0, -1.0 };
			vector lb = { 2.0, -50.0, 0.0 };
			vector ub = { 50.0, 50.0, INFINITY };
			integer m = 1;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			//options.output = true;
			LASO::ProblemAL problem(n, hs021, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			problem.pgPenalties_k[0] = 0.01;
			//solver.solve(problem);

		}

	};
}