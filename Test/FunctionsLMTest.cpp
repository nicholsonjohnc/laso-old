#include "CppUnitTest.h"

#include "SolverAL.h"
#include "FunctionsLM.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(FunctionsLMTest)
	{

	public:

		TEST_METHOD(objFun)
		{

			integer n = 2;
			integer m = 0;
			vector aVector = { 1.0, 2.0 };
			matrix AMatrix = { { 1.0, 2.0 },{ 3.0, 4.0 } };
			LASO::FunctionsLM functionsLM(AMatrix, aVector, n, m);
			vector x = { 1.0, 2.0 };
			vector lb = { -INFINITY, -INFINITY };
			vector ub = { INFINITY, INFINITY };
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, functionsLM, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(48.5, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(objGrad)
		{

			integer n = 2;
			integer m = 0;
			vector aVector = { 1.0, 2.0 };
			matrix AMatrix = { { 1.0, 2.0 },{ 3.0, 4.0 } };
			LASO::FunctionsLM functionsLM(AMatrix, aVector, n, m);
			vector x = { 1.0, 2.0 };
			vector lb = { -INFINITY, -INFINITY };
			vector ub = { INFINITY, INFINITY };
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, functionsLM, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(31.0, problem.objectiveGradient_k[0]);
			Assert::AreEqual(44.0, problem.objectiveGradient_k[1]);

		}

		TEST_METHOD(FunctionsLM)
		{

			integer n = 2;
			integer m = 0;
			vector aVector = { 4.1100559257086866, -23.668463928328105 };
			matrix AMatrix = { { 1.0488338203775793, -0.44649820676022106 },{ -0.44649820676022106, 3.6065414764402748 } };
			LASO::FunctionsLM functionsLM(AMatrix, aVector, n, m);
			vector x = { 0.0, 0.0 };
			vector lb = { -INFINITY, -INFINITY };
			vector ub = { INFINITY, INFINITY };
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, functionsLM, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			//solver.solve(problem);

			//Assert::AreEqual(-17.736512771536113, problem.objectiveGradient_k[0]);
			//Assert::AreEqual(111.53081685968215, problem.objectiveGradient_k[1]);

		}

	};
}