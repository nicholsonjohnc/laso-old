#include "CppUnitTest.h"

#include "SolverAL.h"
#include "SP002.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(SP002Test)
	{

	public:

		TEST_METHOD(fObjEvaluation)
		{

			integer n = 14;
			integer m = 0;
			vector x(n);
			vector lb(n);
			vector ub(n);
			for (int i = 0; i < n; i += 2)
			{
				x[i] = -(double)1.2;
			}
			for (int i = 1; i < n; i += 2)
			{
				x[i] = (double)1;
			}
			for (int i = 0; i < n; ++i)
			{
				lb[i] = -INFINITY;
			}
			for (int i = 0; i < n; ++i)
			{
				ub[i] = INFINITY;
			}
			LASO::SP002 sp002(n, m);
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, sp002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(3073.3999999999996, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
		{

			integer n = 14;
			integer m = 0;
			vector x(n);
			vector lb(n);
			vector ub(n);
			for (int i = 0; i < n; i += 2)
			{
				x[i] = -(double)1.2;
			}
			for (int i = 1; i < n; i += 2)
			{
				x[i] = (double)1;
			}
			for (int i = 0; i < n; ++i)
			{
				lb[i] = -INFINITY;
			}
			for (int i = 0; i < n; ++i)
			{
				ub[i] = INFINITY;
			}
			LASO::SP002 sp002(n, m);
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, sp002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(-215.59999999999991, problem.objectiveGradient_k[0]);
			Assert::AreEqual(792.00000000000000, problem.objectiveGradient_k[1]);
			Assert::AreEqual(-655.59999999999991, problem.objectiveGradient_k[2]);
			Assert::AreEqual(792.00000000000000, problem.objectiveGradient_k[3]);
			Assert::AreEqual(-655.59999999999991, problem.objectiveGradient_k[4]);
			Assert::AreEqual(792.00000000000000, problem.objectiveGradient_k[5]);
			Assert::AreEqual(-655.59999999999991, problem.objectiveGradient_k[6]);
			Assert::AreEqual(792.00000000000000, problem.objectiveGradient_k[7]);
			Assert::AreEqual(-655.59999999999991, problem.objectiveGradient_k[8]);
			Assert::AreEqual(792.00000000000000, problem.objectiveGradient_k[9]);
			Assert::AreEqual(-655.59999999999991, problem.objectiveGradient_k[10]);
			Assert::AreEqual(792.00000000000000, problem.objectiveGradient_k[11]);
			Assert::AreEqual(-655.59999999999991, problem.objectiveGradient_k[12]);
			Assert::AreEqual(-87.999999999999986, problem.objectiveGradient_k[13]);

		}

		TEST_METHOD(SP002)
		{

			integer n = 14;
			integer m = 0;
			vector x(n);
			vector lb(n);
			vector ub(n);
			for (int i = 0; i < n; i += 2)
			{
				x[i] = -(double)1.2;
			}
			for (int i = 1; i < n; i += 2)
			{
				x[i] = (double)1;
			}
			for (int i = 0; i < n; ++i)
			{
				lb[i] = -INFINITY;
			}
			for (int i = 0; i < n; ++i)
			{
				ub[i] = INFINITY;
			}
			LASO::SP002 sp002(n, m);
			LASO::Options options;
			options.majorIterationsLimit = 5000;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			options.output = true;
			LASO::ProblemAL problem(n, sp002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.solve(problem);

		}

	};
}