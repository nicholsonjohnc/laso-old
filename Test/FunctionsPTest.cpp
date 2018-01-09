#include "CppUnitTest.h"

#include "SolverAL.h"
#include "FunctionsP.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(FunctionsPTest)
	{

	public:

		TEST_METHOD(objFun)
		{

			integer n = 3;
			integer m = 0;
			vector CMatrix = { 1.0, 2.0 };
			vector cVector = { 3.0 };
			LASO::FunctionsP functionsP(CMatrix, cVector, n, m);
			vector x = { 4.0, 5.0, 6.0 };
			vector lb = { 0.0, 0.0, -INFINITY };
			vector ub = { INFINITY, INFINITY, INFINITY };
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, functionsP, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(400.5, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(objGrad)
		{

			integer n = 3;
			integer m = 0;
			vector CMatrix = { 1.0, 2.0 };
			vector cVector = { 3.0 };
			LASO::FunctionsP functionsP(CMatrix, cVector, n, m);
			vector x = { 4.0, 5.0, 6.0 };
			vector lb = { 0.0, 0.0, -INFINITY };
			vector ub = { INFINITY, INFINITY, INFINITY };
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, functionsP, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(39.0, problem.objectiveGradient_k[0]);
			Assert::AreEqual(66.0, problem.objectiveGradient_k[1]);
			Assert::AreEqual(58.0, problem.objectiveGradient_k[2]);

		}

	};
}