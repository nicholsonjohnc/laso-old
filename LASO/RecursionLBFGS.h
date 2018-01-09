#pragma once

#ifndef LASO_RECURSIONLBFGS
#define LASO_RECURSIONLBFGS

#include "Common.h"

namespace LASO
{
	// Declare RecursionLBFGS class. 
	class RecursionLBFGS
	{

		// Declare public attributes.
	public:

		// Default constructor.
		RecursionLBFGS();

		// Function that evaluates rVector = Hg, where H represents the l-bfgs approximation of the inverse Hessian and g represents some vector, typically the gradient.	
		void evaluateRVector(matrix &sVectors, matrix &yVectors,
			vector &gradient, vectorinteger &components, integer noComponents,
			integer kIteration, integer noCorrections, vectorinteger &correctionHistory,
			vector &qVector, vector &scaling, vector &rVector);

		// Set qVector = gradient.
		void setQVector(vector &qVector, vector &gradient,
			vectorinteger &components, integer noComponents);

		// Return limit = 0 if iteration < noCorrections.  Return limit = iteration - noCorrections if iteration >= noCorrections.
		integer returnLimit(integer currentIteration, integer noCorrections);

		// Evaluate rho_i = 1.0 / sdoty.
		void evaluateAndStoreRho(vector &rho, integer correctionNo,
			vectorinteger &components, integer noComponents, matrix &sVectors, matrix &yVectors);

		// Evaluate alpha_i = rho_i * (sdotq). 
		void evaluateAndStoreAlpha(vector &alpha, vector &rho,
			integer counter, vector &qVector, integer correctionNo,
			vectorinteger &components, integer noComponents, matrix &sVectors);

		// Update qVector = qVector - alpha_i * y.
		void updateQVector(vector &qVector, vector &alpha,
			integer counter, integer correctionNo, vectorinteger &components, integer noComponents,
			matrix &yVectors);

		// Return H0 = sdoty / ydoty.
		doublereal returnH0Factor(integer correctionNo, vectorinteger &components,
			integer noComponents, matrix &sVectors, matrix &yVectors);

		// Return restricted H0 between 0.001 and 1000 so long as positive.
		doublereal returnRestrictedH0Factor(doublereal H0);

		// Set rVector = H0 * qVector.
		void setRVector(vector &rVector, doublereal H0,
			vector &qVector, vectorinteger &components, integer noComponents);

		// Return betaVariable = rho_i * (ydotr).
		doublereal returnBetaVariable(vector &rho, integer counter,
			vector &rVector, vectorinteger &components, integer noComponents,
			integer correctionNo, matrix &yVectors);

		// Update rVector = rVector + s * (alpha_i - betaVariable).
		void updateRVector(vector &rVector, vectorinteger &components,
			integer noComponents, integer correctionNo, matrix &sVectors, vector &alpha,
			integer counter, doublereal betaVariable);

		// Declare private attributes.
	private:

	};

}

#endif