#include "CppUnitTest.h"

#include "SolverAL.h"
#include "POWELLBADLYSCALED.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(POWELLBADLYSCALEDTest)
	{

	public:

		//TEST_METHOD(fObjEvaluation)
		//{

		//	integer n = 3;
		//	integer m = 0;
		//	vector x(n);
		//	vector lb(n + m);
		//	vector ub(n + m);
		//	x[0] = 1.0;
		//	x[1] = 2.0;
		//	x[2] = 3.0;
		//	lb[0] = -INFINITY;
		//	lb[1] = -INFINITY;
		//	lb[2] = -INFINITY;
		//	ub[0] = INFINITY;
		//	ub[1] = INFINITY;
		//	ub[2] = INFINITY;
		//	LASO::SP007 sp007;
		//	LASO::Options options;
		//	vector fObj(1);
		//	vector gObj(n);
		//	vector fCon(1);
		//	vector gCon(1);
		//	LASO::ProblemAL problem(n, sp007, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
		//	Assert::AreEqual(315.03052382223188, problem.fObjectiveFunction_k[0]);

		//}

		//TEST_METHOD(gObjEvaluation)
		//{

		//	integer n = 3;
		//	integer m = 0;
		//	vector x(n);
		//	vector lb(n + m);
		//	vector ub(n + m);
		//	x[0] = 1.0;
		//	x[1] = 2.0;
		//	x[2] = 3.0;
		//	lb[0] = -INFINITY;
		//	lb[1] = -INFINITY;
		//	lb[2] = -INFINITY;
		//	ub[0] = INFINITY;
		//	ub[1] = INFINITY;
		//	ub[2] = INFINITY;
		//	LASO::SP007 sp007;
		//	LASO::Options options;
		//	vector fObj(1);
		//	vector gObj(n);
		//	vector fCon(1);
		//	vector gCon(1);
		//	LASO::ProblemAL problem(n, sp007, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
		//	problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
		//	Assert::AreEqual(268.17390721056955, problem.objectiveGradient_k[0]);
		//	Assert::AreEqual(142.30624864473629, problem.objectiveGradient_k[1]);
		//	Assert::AreEqual(253.58361765043327, problem.objectiveGradient_k[2]);

		//}

		TEST_METHOD(POWELLBADLYSCALED)
		{

			integer n = 2;
			integer m = 0;
			vector x(n);
			vector lb(n + m);
			vector ub(n + m);
			x[0] = 0.0;
			x[1] = 1.0;
			lb[0] = -10.0;
			lb[1] = -10.0;
			ub[0] = 10.0;
			ub[1] = 10.0;
			LASO::POWELLBADLYSCALED powellBadlyScaled;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			//options.output = true;
			options.scaling = 1E-9;
			LASO::ProblemAL problem(n, powellBadlyScaled, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			//solver.solve(problem);

		}

	};
}