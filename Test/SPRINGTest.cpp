#include "CppUnitTest.h"

#include "SolverAL.h"
#include "SPRING.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(SPRINGTest)
	{

	public:

		TEST_METHOD(fObjEvaluation)
		{

			integer n = 3;
			LASO::SPRING spring;
			vector x = { 0.2, 1.3, 2.0 };
			vector lb = { 0.05, 0.25, 2.0, -INFINITY, -INFINITY, -INFINITY, -INFINITY };
			vector ub = { 0.20, 1.3, 15.0, 0.0, 0.0, 0.0, 0.0 };
			integer m = 4;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			options.output = true;
			LASO::ProblemAL problem(n, spring, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objFun(problem.xDesignVariables_k, problem.fObjectiveFunction_k);
			Assert::AreEqual(0.20800000000000005, problem.fObjectiveFunction_k[0]);

		}

		TEST_METHOD(gObjEvaluation)
		{

			integer n = 3;
			LASO::SPRING spring;
			vector x = { 0.2, 1.3, 2.0 };
			vector lb = { 0.05, 0.25, 2.0, -INFINITY, -INFINITY, -INFINITY, -INFINITY };
			vector ub = { 0.20, 1.3, 15.0, 0.0, 0.0, 0.0, 0.0 };
			integer m = 4;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			options.output = true;
			LASO::ProblemAL problem(n, spring, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);
			Assert::AreEqual(2.0800000000000001, problem.objectiveGradient_k[0]);
			Assert::AreEqual(0.16000000000000003, problem.objectiveGradient_k[1]);
			Assert::AreEqual(0.052000000000000011, problem.objectiveGradient_k[2]);

		}

		TEST_METHOD(fConEvaluation)
		{

			integer n = 3;
			LASO::SPRING spring;
			vector x = { 0.2, 1.3, 2.0 };
			vector lb = { 0.05, 0.25, 2.0, -INFINITY, -INFINITY, -INFINITY, -INFINITY };
			vector ub = { 0.20, 1.3, 15.0, 0.0, 0.0, 0.0, 0.0 };
			integer m = 4;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			options.output = true;
			LASO::ProblemAL problem(n, spring, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conFun(problem.xDesignVariables_k, problem.fConstraintFunctions_k);
			Assert::AreEqual(0.96179130434782611, problem.fConstraintFunctions_k[0]);
			Assert::AreEqual(-0.93632529336014936, problem.fConstraintFunctions_k[1]);
			Assert::AreEqual(-7.3159763313609467, problem.fConstraintFunctions_k[2]);
			Assert::AreEqual(0.0, problem.fConstraintFunctions_k[3]);

		}

		TEST_METHOD(jConEvaluation)
		{

			integer n = 3;
			LASO::SPRING spring;
			vector x = { 0.2, 1.3, 2.0 };
			vector lb = { 0.05, 0.25, 2.0, -INFINITY, -INFINITY, -INFINITY, -INFINITY };
			vector ub = { 0.20, 1.3, 15.0, 0.0, 0.0, 0.0, 0.0 };
			integer m = 4;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			options.output = true;
			LASO::ProblemAL problem(n, spring, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.conJac(problem.xDesignVariables_k, problem.jConstraintJacobian_k);
			
			Assert::AreEqual(0.76417391304347804, problem.jConstraintJacobian_k[0 * m + 0]);
			Assert::AreEqual(-0.088173913043478255, problem.jConstraintJacobian_k[1 * m + 0]);
			Assert::AreEqual(-0.019104347826086953, problem.jConstraintJacobian_k[2 * m + 0]);

			Assert::AreEqual(-0.88896905331584752, problem.jConstraintJacobian_k[0 * m + 1]);
			Assert::AreEqual(0.038803382602667823, problem.jConstraintJacobian_k[1 * m + 1]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[2 * m + 1]);

			Assert::AreEqual(-41.579881656804730, problem.jConstraintJacobian_k[0 * m + 2]);
			Assert::AreEqual(12.793809740555302, problem.jConstraintJacobian_k[1 * m + 2]);
			Assert::AreEqual(4.1579881656804734, problem.jConstraintJacobian_k[2 * m + 2]);

			Assert::AreEqual(0.66666666666666663, problem.jConstraintJacobian_k[0 * m + 3]);
			Assert::AreEqual(0.66666666666666663, problem.jConstraintJacobian_k[1 * m + 3]);
			Assert::AreEqual(0.0, problem.jConstraintJacobian_k[2 * m + 3]);

		}

		TEST_METHOD(SPRING)
		{

			integer n = 3;
			LASO::SPRING spring;
			vector x = { 0.2, 1.3, 2.0 };
			vector lb = { 0.05, 0.25, 2.0, -INFINITY, -INFINITY, -INFINITY, -INFINITY };
			vector ub = { 0.20, 1.3, 15.0, 0.0, 0.0, 0.0, 0.0 };
			integer m = 4;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(m);
			vector gCon(m * n);
			options.output = true;
			LASO::ProblemAL problem(n, spring, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			LASO::SolverAL solver;
			solver.solve(problem);

		}

	};
}