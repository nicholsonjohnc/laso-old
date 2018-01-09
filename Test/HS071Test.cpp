#include "CppUnitTest.h"

#include "SolverAL.h"
#include "HS071.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(HS071Test)
	{

	public:

		TEST_METHOD(fObjEvaluation)
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

			problem.xDesignVariables_k[0] = 1.0;
			problem.xDesignVariables_k[1] = 4.7429994;
			problem.xDesignVariables_k[2] = 3.8211503;
			problem.xDesignVariables_k[3] = 1.3794082;
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(17.014016822207545, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
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
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(12.0, problem.objectiveGradient_k[0]);
			Assert::AreEqual(1.0, problem.objectiveGradient_k[1]);
			Assert::AreEqual(2.0, problem.objectiveGradient_k[2]);
			Assert::AreEqual(11.0, problem.objectiveGradient_k[3]);

		}

		TEST_METHOD(fConEvaluation)
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
			problem.functions.conFun(problem.xDesignVariables_k, problem.fConstraintFunctions_k);
			Assert::AreEqual(0.0, problem.fConstraintFunctions_k[0]);
			Assert::AreEqual(12.0, problem.fConstraintFunctions_k[1]);

		}

		TEST_METHOD(jConEvaluation)
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

		TEST_METHOD(HS071)
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
			//options.output = true;
			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.pgPenalties_k[0] = 0.1;
			problem.phPenalties_k[0] = 0.1;
			LASO::SolverAL solver;
			//solver.solve(problem);

		}

	};
}