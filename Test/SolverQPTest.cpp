//#include "CppUnitTest.h"
//
//#include "SolverQP.h"
//
//using namespace Microsoft::VisualStudio::CppUnitTestFramework;
//
//namespace LASO
//{
//	TEST_CLASS(SolverQPTest)
//	{
//
//	public:
//
//		TEST_METHOD(evaluateBVector)
//		{
//
//			integer n = 4;
//			LASO::HS071 hs071;
//			vector x = { 1.0, 5.0, 5.0, 1.0 };
//			vector lb = { 1.0, 1.0, 1.0, 1.0, 0.0, 0.0 };
//			vector ub = { 5.0, 5.0, 5.0, 5.0, INFINITY, 0.0 };
//			integer m = 2;
//			LASO::Options options;
//			vector fObj(1);
//			vector gObj(n);
//			vector fCon(m);
//			vector gCon(m * n);
//			LASO::ProblemAL problem(n, hs071, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
//			LASO::SolverAL solver;
//			solver.evaluateObjectiveGradient(problem.xDesignVariables_k,
//				problem.objectiveGradient_k, problem);
//			solver.evaluateAuxiliaryConstraints(problem.xDesignVariables_k,
//				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k, problem);
//			solver.evaluateAuxiliaryJacobians(problem.xDesignVariables_k,
//				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k, problem);
//			problem.phPenalties_k[0] = 1.0;
//			problem.pgPenalties_k[0] = 1.0;
//			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
//				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
//				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
//				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
//				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
//				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
//				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
//				problem);
//			solver.identifyActiveInactiveSet(problem.xDesignVariables_k,
//				problem.zSlackVariables_k, problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
//				problem.dxSearchDirection_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
//				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
//				problem.inactiveVariableSet_k, problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
//				problem);
//			solver.evaluateBVector(problem.kIterationSinceRestart,
//				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
//				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
//				problem.augmentedLagrangianGradientX_k, problem.sVectors, problem.yVectors,
//				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
//				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
//				problem.correctionHistory, problem.options.noCorrections,
//				problem.bVector_k, problem);
//
//			Assert::AreEqual(-2418.0, problem.bVector_k[0]);
//			//Assert::AreEqual(1215.0, problem.bVector_k[1]);
//
//			solver.evaluateInactiveDesignVariableSearchDirection(problem.kIterationSinceRestart,
//				problem.augmentedLagrangianGradientX_k, problem.sVectors, problem.yVectors,
//				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
//				problem.correctionHistory, problem.options.noCorrections,
//				problem.dxSearchDirection_k, problem);
//			solver.ensureInactiveSearchDirectionThatOfDescent(problem.dxSearchDirection_k,
//				problem.augmentedLagrangianGradientX_k, problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
//				problem.correctionHistory, problem);
//			solver.evaluatePhiPrime0LineSearchFunctionGradient(
//				problem.augmentedLagrangianGradientX_k, problem.dxSearchDirection_k,
//				problem.augmentedLagrangianGradientZ_k, problem.dzSearchDirection_k,
//				problem.augmentedLagrangianGradientMu_k, problem.dmuSearchDirection_k,
//				problem.augmentedLagrangianGradientLambda_k, problem.dlambdaSearchDirection_k,
//				problem.phiPrime0_k, problem);
//			solver.identifyStepSize(problem.alphaStepSize_k, problem.kIterationSinceRestart,
//				problem.xDesignVariables_k, problem.zSlackVariables_k,
//				problem.muLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_k,
//				problem.dxSearchDirection_k, problem.dzSearchDirection_k,
//				problem.dmuSearchDirection_k, problem.dlambdaSearchDirection_k,
//				problem.phPenalties_k, problem.pgPenalties_k,
//				problem.fObjectiveFunction_k,
//				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
//				problem.phiPrime0_k,
//				problem.xDesignVariables_kPlus1, problem.zSlackVariables_kPlus1,
//				problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
//				problem.fObjectiveFunction_kPlus1,
//				problem.hConstraintFunctions_kPlus1, problem.gConstraintFunctions_kPlus1,
//				problem);
//			solver.evaluateObjectiveGradient(&problem.xDesignVariables_kPlus1[0],
//				&problem.objectiveGradient_kPlus1[0], problem);
//			solver.evaluateAuxiliaryJacobians(&problem.xDesignVariables_kPlus1[0],
//				problem.jhConstraintJacobian_kPlus1, problem.jgConstraintJacobian_kPlus1, problem);
//			solver.evaluateAugmentedLagrangianGradients(problem.objectiveGradient_k,
//				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
//				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
//				problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
//				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_k,
//				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
//				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientLambda_k,
//				problem);
//			solver.evaluateAugmentedLagrangianGradients(&problem.objectiveGradient_kPlus1[0],
//				problem.hConstraintFunctions_kPlus1, problem.gConstraintFunctions_kPlus1,
//				problem.jhConstraintJacobian_kPlus1, problem.jgConstraintJacobian_kPlus1,
//				problem.muLagrangeMultipliers_kPlus1, problem.lambdaLagrangeMultipliers_kPlus1,
//				problem.phPenalties_k, problem.pgPenalties_k, problem.zSlackVariables_kPlus1,
//				problem.augmentedLagrangianGradientX_kPlus1, problem.augmentedLagrangianGradientZ_kPlus1,
//				problem.augmentedLagrangianGradientMu_kPlus1, problem.augmentedLagrangianGradientLambda_kPlus1,
//				problem);
//			solver.updateCorrectionVectors(problem.xDesignVariables_k, problem.xDesignVariables_kPlus1,
//				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientX_kPlus1,
//				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
//				problem.correctionHistory, problem.options.noCorrections,
//				problem.sVectors, problem.yVectors,
//				problem);
//			solver.setVariablesForNextIteration(
//				problem.fObjectiveFunction_k, problem.fObjectiveFunction_kPlus1,
//				problem.xDesignVariables_k, problem.xDesignVariables_kPlus1,
//				problem.zSlackVariables_k, problem.zSlackVariables_kPlus1,
//				problem.muLagrangeMultipliers_k, problem.muLagrangeMultipliers_kPlus1,
//				problem.lambdaLagrangeMultipliers_k, problem.lambdaLagrangeMultipliers_kPlus1,
//				problem.hConstraintFunctions_k, problem.hConstraintFunctions_kPlus1,
//				problem.gConstraintFunctions_k, problem.gConstraintFunctions_kPlus1,
//				problem.objectiveGradient_k, problem.objectiveGradient_kPlus1,
//				problem.jhConstraintJacobian_k, problem.jhConstraintJacobian_kPlus1,
//				problem.jgConstraintJacobian_k, problem.jgConstraintJacobian_kPlus1,
//				problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientX_kPlus1,
//				problem.augmentedLagrangianGradientZ_k, problem.augmentedLagrangianGradientZ_kPlus1,
//				problem.augmentedLagrangianGradientMu_k, problem.augmentedLagrangianGradientMu_kPlus1,
//				problem.augmentedLagrangianGradientLambda_k, problem.augmentedLagrangianGradientLambda_kPlus1,
//				problem.lagrangianGradient_k, problem.lagrangianGradient_kPlus1,
//				problem);
//			++problem.kIteration;
//			++problem.kIterationSinceRestart;
//			solver.identifyActiveInactiveSet(problem.xDesignVariables_k,
//				problem.zSlackVariables_k, problem.augmentedLagrangianGradientX_k, problem.augmentedLagrangianGradientZ_k,
//				problem.dxSearchDirection_k, problem.dzSearchDirection_k, problem.dlambdaSearchDirection_k,
//				problem.lambdaLagrangeMultipliersL_k, problem.lambdaLagrangeMultipliersU_k, problem.lambdaLagrangeMultipliers_k,
//				problem.inactiveVariableSet_k, problem.activeConstraintSet_k, problem.inactiveConstraintSet_k,
//				problem);
//			solver.evaluateBMatrix(problem.kIterationSinceRestart,
//				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
//				problem.sVectors, problem.yVectors,
//				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
//				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
//				problem.correctionHistory, problem.options.noCorrections,
//				problem.BMatrix_k, problem);
//			solver.evaluateBVector(problem.kIterationSinceRestart,
//				problem.hConstraintFunctions_k, problem.gConstraintFunctions_k,
//				problem.jhConstraintJacobian_k, problem.jgConstraintJacobian_k,
//				problem.augmentedLagrangianGradientX_k, problem.sVectors, problem.yVectors,
//				problem.inactiveVariableSet_k, problem.nINoInactiveVariables_k,
//				problem.activeConstraintSet_k, problem.mANoActiveConstraints_k,
//				problem.correctionHistory, problem.options.noCorrections,
//				problem.bVector_k, problem);
//
//			Assert::AreEqual(-0.11581251098102197, problem.bVector_k[0]);
//			Assert::AreEqual(-0.85310462382124186, problem.bVector_k[1]);
//
//		}
//
//	};
//
//}