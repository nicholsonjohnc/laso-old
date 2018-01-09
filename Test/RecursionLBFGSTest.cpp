#include "CppUnitTest.h"

#include "HS002.h"
#include "HS071.h"
#include "ProblemAL.h"
#include "RecursionLBFGS.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(RecursionLBFGSTest)
	{

	public:

		// Set qVector = gradient.
		TEST_METHOD(setQVectorComponentsToGradient)
		{

			integer n = 2;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0 };
			vector lb = { -INFINITY, 1.5 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);

			// Test dxSearchDirection calculated correctly on second iteration.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			LASO::RecursionLBFGS recursionLBFGS;
			integer correctionNo = 0;
			vector rho_i;
			vector alpha_i;
			integer counter = 0;
			recursionLBFGS.setQVector(problem.qVector, gObj, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			Assert::AreEqual(-2406.0, problem.qVector[0]);
			Assert::AreEqual(-600.0, problem.qVector[1]);

		}

		// Return limit = 0 if iteration < noCorrections.  Return limit = iteration - noCorrections if iteration >= noCorrections.
		TEST_METHOD(returnLimit)
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
			integer limit = 0;
			integer currentIteration = 0;
			LASO::RecursionLBFGS recursionLBFGS;
			limit = recursionLBFGS.returnLimit(currentIteration, problem.options.noCorrections);
			Assert::AreEqual((integer)0, limit);
			currentIteration = 25;
			limit = recursionLBFGS.returnLimit(currentIteration, problem.options.noCorrections);
			Assert::AreEqual((integer)5, limit);

		}

		// Evaluate rho_i = 1.0 / sdoty.
		TEST_METHOD(evaluateAndStoreRho)
		{
			integer n = 2;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0 };
			vector lb = { -INFINITY, 1.5 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);

			// Test dxSearchDirection calculated correctly on second iteration.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			LASO::RecursionLBFGS recursionLBFGS;
			integer correctionNo = 0;
			vector rho_i;
			vector alpha_i;
			integer counter = 0;
			recursionLBFGS.setQVector(problem.qVector, gObj, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			recursionLBFGS.evaluateAndStoreRho(rho_i, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			Assert::AreEqual(0.00020798668885191348, rho_i[0]);
		}

		// Evaluate alpha_i = rho_i * (sdotq).
		TEST_METHOD(evaluateAndStoreAlpha)
		{
			integer n = 2;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0 };
			vector lb = { -INFINITY, 1.5 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);

			// Test dxSearchDirection calculated correctly on second iteration.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			LASO::RecursionLBFGS recursionLBFGS;
			integer correctionNo = 0;
			vector rho_i;
			vector alpha_i;
			integer counter = 0;
			recursionLBFGS.setQVector(problem.qVector, gObj, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			recursionLBFGS.evaluateAndStoreRho(rho_i, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			recursionLBFGS.evaluateAndStoreAlpha(alpha_i, rho_i, counter, problem.qVector, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors);
			Assert::AreEqual(-1.0008319467554077, alpha_i[0]);
		}

		// Update qVector = qVector - alpha_i * y.
		TEST_METHOD(updateQVectorComponents)
		{
			integer n = 2;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0 };
			vector lb = { -INFINITY, 1.5 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);

			// Test dxSearchDirection calculated correctly on second iteration.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			LASO::RecursionLBFGS recursionLBFGS;
			integer correctionNo = 0;
			vector rho_i;
			vector alpha_i;
			integer counter = 0;
			recursionLBFGS.setQVector(problem.qVector, gObj, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			recursionLBFGS.evaluateAndStoreRho(rho_i, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			recursionLBFGS.evaluateAndStoreAlpha(alpha_i, rho_i, counter, problem.qVector, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors);
			recursionLBFGS.updateQVector(problem.qVector, alpha_i, counter, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.yVectors);
			Assert::AreEqual(0.00000000000000000, problem.qVector[0]);
			Assert::AreEqual(200.66555740432614, problem.qVector[1]);
		}

		// Return H0 = sdoty / ydoty.
		TEST_METHOD(returnH0DynamicScalingFactor)
		{
			integer n = 2;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0 };
			vector lb = { -INFINITY, 1.5 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);

			// Test dxSearchDirection calculated correctly on second iteration.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			LASO::RecursionLBFGS recursionLBFGS;
			integer correctionNo = 0;
			vector rho_i;
			vector alpha_i;
			integer counter = 0;
			doublereal H0 = 0.0;
			recursionLBFGS.setQVector(problem.qVector, gObj, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			recursionLBFGS.evaluateAndStoreRho(rho_i, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			recursionLBFGS.evaluateAndStoreAlpha(alpha_i, rho_i, counter, problem.qVector, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors);
			recursionLBFGS.updateQVector(problem.qVector, alpha_i, counter, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.yVectors);
			H0 = recursionLBFGS.returnH0Factor(correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			Assert::AreEqual(0.00074900112412481525, H0);
		}

		// Return restricted H0 between 0.001 and 1000 so long as positive.
		TEST_METHOD(returnRestrictedH0Factor)
		{
			integer n = 2;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0 };
			vector lb = { -INFINITY, 1.5 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);

			// Test dxSearchDirection calculated correctly on second iteration.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			LASO::RecursionLBFGS recursionLBFGS;
			integer correctionNo = 0;
			vector rho_i;
			vector alpha_i;
			integer counter = 0;
			doublereal H0 = 0.0;
			recursionLBFGS.setQVector(problem.qVector, gObj, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			recursionLBFGS.evaluateAndStoreRho(rho_i, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			recursionLBFGS.evaluateAndStoreAlpha(alpha_i, rho_i, counter, problem.qVector, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors);
			recursionLBFGS.updateQVector(problem.qVector, alpha_i, counter, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.yVectors);
			H0 = recursionLBFGS.returnH0Factor(correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			H0 = recursionLBFGS.returnRestrictedH0Factor(H0);
			Assert::AreEqual(0.001, H0);
			H0 = 1001.0;
			H0 = recursionLBFGS.returnRestrictedH0Factor(H0);
			Assert::AreEqual(1000.0, H0);
			H0 = 1.0;
			H0 = recursionLBFGS.returnRestrictedH0Factor(H0);
			Assert::AreEqual(1.0, H0);
		}

		// Set rVector = H0 * qVector.
		TEST_METHOD(setRVectorComponents)
		{
			integer n = 2;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0 };
			vector lb = { -INFINITY, 1.5 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);

			// Test dxSearchDirection calculated correctly on second iteration.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			LASO::RecursionLBFGS recursionLBFGS;
			integer correctionNo = 0;
			vector rho_i;
			vector alpha_i;
			integer counter = 0;
			doublereal H0 = 0.0;
			recursionLBFGS.setQVector(problem.qVector, gObj, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			recursionLBFGS.evaluateAndStoreRho(rho_i, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			recursionLBFGS.evaluateAndStoreAlpha(alpha_i, rho_i, counter, problem.qVector, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors);
			recursionLBFGS.updateQVector(problem.qVector, alpha_i, counter, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.yVectors);
			H0 = recursionLBFGS.returnH0Factor(correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			H0 = recursionLBFGS.returnRestrictedH0Factor(H0);
			recursionLBFGS.setRVector(problem.rVector, H0, problem.qVector, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			Assert::AreEqual(0.0, problem.rVector[0]);
			Assert::AreEqual(0.20066555740432615, problem.rVector[1]);
		}

		// Return betaVariable = rho_i * (ydotr).
		TEST_METHOD(returnBetaVariable)
		{
			integer n = 2;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0 };
			vector lb = { -INFINITY, 1.5 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);

			// Test dxSearchDirection calculated correctly on second iteration.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			LASO::RecursionLBFGS recursionLBFGS;
			integer correctionNo = 0;
			vector rho_i;
			vector alpha_i;
			integer counter = 0;
			doublereal H0 = 0.0;
			doublereal betaVariable = 0.0;
			recursionLBFGS.setQVector(problem.qVector, gObj, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			recursionLBFGS.evaluateAndStoreRho(rho_i, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			recursionLBFGS.evaluateAndStoreAlpha(alpha_i, rho_i, counter, problem.qVector, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors);
			recursionLBFGS.updateQVector(problem.qVector, alpha_i, counter, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.yVectors);
			H0 = recursionLBFGS.returnH0Factor(correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			H0 = recursionLBFGS.returnRestrictedH0Factor(H0);
			recursionLBFGS.setRVector(problem.rVector, H0, problem.qVector, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			betaVariable = recursionLBFGS.returnBetaVariable(rho_i, counter, problem.rVector, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, correctionNo, problem.yVectors);
			Assert::AreEqual(0.033388611880919496, betaVariable);
		}

		// Update rVector = rVector + s * (alpha_i - betaVariable).
		TEST_METHOD(updateRVectorComponents)
		{
			integer n = 2;
			LASO::HS002 hs002;
			vector x = { -2.0, 1.0 };
			vector lb = { -INFINITY, 1.5 };
			vector ub = { INFINITY, INFINITY };
			integer m = 0;
			LASO::Options options;
			vector fObj(1);
			vector gObj(n);
			vector fCon(1);
			vector gCon(1);
			LASO::ProblemAL problem(n, hs002, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
			problem.functions.objGrad(problem.xDesignVariables_k, problem.objectiveGradient_k);

			// Test dxSearchDirection calculated correctly on second iteration.
			problem.kIteration = 1;
			problem.correctionHistory.clear();
			problem.correctionHistory.push_back(0);
			problem.sVectors[0][0] = 2.0;
			problem.sVectors[0][1] = 0.0;
			problem.yVectors[0][0] = 2404.0;
			problem.yVectors[0][1] = 800.0;
			LASO::RecursionLBFGS recursionLBFGS;
			integer correctionNo = 0;
			vector rho_i;
			vector alpha_i;
			integer counter = 0;
			doublereal H0 = 0.0;
			doublereal betaVariable = 0.0;
			recursionLBFGS.setQVector(problem.qVector, gObj, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			recursionLBFGS.evaluateAndStoreRho(rho_i, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			recursionLBFGS.evaluateAndStoreAlpha(alpha_i, rho_i, counter, problem.qVector, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors);
			recursionLBFGS.updateQVector(problem.qVector, alpha_i, counter, correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.yVectors);
			H0 = recursionLBFGS.returnH0Factor(correctionNo, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, problem.sVectors, problem.yVectors);
			H0 = recursionLBFGS.returnRestrictedH0Factor(H0);
			recursionLBFGS.setRVector(problem.rVector, H0, problem.qVector, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k);
			betaVariable = recursionLBFGS.returnBetaVariable(rho_i, counter, problem.rVector, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, correctionNo, problem.yVectors);
			recursionLBFGS.updateRVector(problem.rVector, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k, correctionNo, problem.sVectors, alpha_i, counter, betaVariable);
			Assert::AreEqual(-2.0684411172726542, problem.rVector[0]);
			Assert::AreEqual(0.20066555740432615, problem.rVector[1]);
		}


	};

}