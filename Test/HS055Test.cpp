#include "CppUnitTest.h"

#include "SolverAL.h"
#include "HS055.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(HS055Test)
	{

	public:

		TEST_METHOD(fObjEvaluation)
		{

			integer n = 6;
			LASO::HS055 hs055;
			vector x = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
			vector lb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			vector ub = { 1.0, INFINITY, INFINITY, 1.0, INFINITY, INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			integer m = 6;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs055, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(79.598150033144236, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
		{

			integer n = 6;
			LASO::HS055 hs055;
			vector x = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
			vector lb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			vector ub = { 1.0, INFINITY, INFINITY, 1.0, INFINITY, INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			integer m = 6;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs055, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(219.39260013257694, problem.objectiveGradient_k[0]);
			Assert::AreEqual(2.0, problem.objectiveGradient_k[1]);
			Assert::AreEqual(0.0, problem.objectiveGradient_k[2]);
			Assert::AreEqual(54.598150033144236, problem.objectiveGradient_k[3]);
			Assert::AreEqual(4.0, problem.objectiveGradient_k[4]);
			Assert::AreEqual(0.0, problem.objectiveGradient_k[5]);

		}

		TEST_METHOD(fConEvaluation)
		{

			integer n = 6;
			LASO::HS055 hs055;
			vector x = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
			vector lb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			vector ub = { 1.0, INFINITY, INFINITY, 1.0, INFINITY, INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			integer m = 6;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs055, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conFun(problem.xDesignVariables_k, problem.fConstraintFunctions_k);
			Assert::AreEqual(24.0, problem.fConstraintFunctions_k[0]);
			Assert::AreEqual(3.0, problem.fConstraintFunctions_k[1]);
			Assert::AreEqual(13.0, problem.fConstraintFunctions_k[2]);
			Assert::AreEqual(4.0, problem.fConstraintFunctions_k[3]);
			Assert::AreEqual(5.0, problem.fConstraintFunctions_k[4]);
			Assert::AreEqual(7.0, problem.fConstraintFunctions_k[5]);

		}

		TEST_METHOD(jConEvaluation)
		{

			integer n = 6;
			LASO::HS055 hs055;
			vector x = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
			vector lb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			vector ub = { 1.0, INFINITY, INFINITY, 1.0, INFINITY, INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			integer m = 6;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			LASO::ProblemAL problem(n, hs055, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conJac(problem.xDesignVariables_k, problem.jConstraintJacobian_k);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[0 * m + 0]);
			Assert::AreEqual(2.0, problem.jConstraintJacobian_k[1 * m + 0]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[2 * m + 0]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[3 * m + 0]);
			Assert::AreEqual(5.0, problem.jConstraintJacobian_k[4 * m + 0]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[5 * m + 0]);

			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[0 * m + 1]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[1 * m + 1]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[2 * m + 1]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[3 * m + 1]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[4 * m + 1]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[5 * m + 1]);

			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[0 * m + 2]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[1 * m + 2]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[2 * m + 2]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[3 * m + 2]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[4 * m + 2]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[5 * m + 2]);

			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[0 * m + 3]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[1 * m + 3]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[2 * m + 3]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[3 * m + 3]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[4 * m + 3]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[5 * m + 3]);

			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[0 * m + 4]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[1 * m + 4]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[2 * m + 4]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[3 * m + 4]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[4 * m + 4]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[5 * m + 4]);

			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[0 * m + 5]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[1 * m + 5]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[2 * m + 5]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[3 * m + 5]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[4 * m + 5]);
			Assert::AreEqual(1.0, problem.jConstraintJacobian_k[5 * m + 5]);

		}

		TEST_METHOD(HS055)
		{

			integer n = 6;
			LASO::HS055 hs055;
			vector x = { 1.0, 2.0, 0.0, 0.0, 0.0, 2.0 };
			vector lb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			vector ub = { 1.0, INFINITY, INFINITY, 1.0, INFINITY, INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			integer m = 6;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			//options.output = true;
			LASO::ProblemAL problem(n, hs055, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			problem.phPenalties_k[0] = 1.0;
			problem.phPenalties_k[1] = 1.0;
			problem.phPenalties_k[2] = 1.0;
			problem.phPenalties_k[3] = 1.0;
			problem.phPenalties_k[4] = 1.0;
			problem.phPenalties_k[5] = 1.0;
			//solver.solve(problem);

			//integer n = 6;
			//LASO::HS055 hs055;
			//vector x = { 0.0, 4.0 / 3.0, 5.0 / 3.0, 1.0, 2.0 / 3.0, 1.0 / 3.0 };
			//vector lb = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			//vector ub = { 1.0, INFINITY, INFINITY, 1.0, INFINITY, INFINITY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			//integer m = 6;
			//LASO::Options options;
			//options.output = true;
			//LASO::ProblemAL problem(n, hs055, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			//LASO::SolverAL solver;
			//problem.phPenalties_k[0] = 10.0;
			//problem.phPenalties_k[1] = 10.0;
			//problem.phPenalties_k[2] = 10.0;
			//problem.phPenalties_k[3] = 10.0;
			//problem.phPenalties_k[4] = 10.0;
			//problem.phPenalties_k[5] = 10.0;
			//solver.solve(problem);

		}

	};
}