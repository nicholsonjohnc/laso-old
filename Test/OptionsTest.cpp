#include "CppUnitTest.h"

#include "Options.h"
#include "HS071.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace LASO
{
	TEST_CLASS(OptionsTest)
	{

	public:

		TEST_METHOD(majorOptimalityToleranceInitialization)
		{

			LASO::Options options;
			Assert::AreEqual(0.000000011, options.majorOptimalityTolerance);

		}

		TEST_METHOD(minorOptimalityToleranceInitialization)
		{

			LASO::Options options;
			Assert::AreEqual(0.000000011, options.minorOptimalityTolerance);

		}

		TEST_METHOD(epsilonFeasibilityInitialization)
		{

			LASO::Options options;
			Assert::AreEqual(0.000000011, options.majorFeasibilityTolerance);

		}

		TEST_METHOD(minorFeasibilityToleranceInitialization)
		{

			LASO::Options options;
			Assert::AreEqual(0.000000011, options.minorFeasibilityTolerance);

		}

		TEST_METHOD(rhoFactorInitialization)
		{

			LASO::Options options;
			Assert::AreEqual(0.0001, options.rhoFactor);

		}

		TEST_METHOD(outputInitialization)
		{

			LASO::Options options;
			Assert::AreEqual(false, options.output);

		}

		TEST_METHOD(majorIterationsLimitInitialization)
		{

			LASO::Options options;
			Assert::AreEqual((integer)500, options.majorIterationsLimit);

		}

		TEST_METHOD(minorIterationsLimitInitialization)
		{

			LASO::Options options;
			Assert::AreEqual((integer)500, options.minorIterationsLimit);

		}

		TEST_METHOD(noCorrectionsInitialization)
		{

			LASO::Options options;
			Assert::AreEqual((integer)20, options.noCorrections);

		}

		TEST_METHOD(penaltyInitialization)
		{

			LASO::Options options;
			Assert::AreEqual(0.01, options.penalty);

		}

		TEST_METHOD(scalingInitialization)
		{

			LASO::Options options;
			Assert::AreEqual(1.0, options.scaling);

		}

		TEST_METHOD(infiniteBoundSizeInitialization)
		{

			LASO::Options options;
			Assert::AreEqual(1.0e20, options.infiniteBoundSize);

		}

	};

}