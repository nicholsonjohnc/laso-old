#include "RecursionLBFGS.h"

// Define default constructor for RecursionLBFGS class. 
LASO::RecursionLBFGS::RecursionLBFGS()
{

}

// Function that evaluates rVector = Hg, where H represents the l-bfgs approximation of the inverse Hessian and g represents some vector, typically the gradient.	
void LASO::RecursionLBFGS::evaluateRVector(matrix &sVectors, matrix &yVectors,
	vector &gradient, vectorinteger &components, integer noComponents,
	integer kIteration, integer noCorrections, vectorinteger &correctionHistory,
	vector &qVector, vector &scaling, vector &rVector)
{
	integer limit = 0;
	integer counter = 0;
	integer correctionNo = 0;
	vector rhoValues;
	vector alphaValues;
	doublereal betaVariable = 0.0;

	// Set qVector = gradient.
	setQVector(qVector, gradient, components, noComponents);

	// Return limit = 0 if kIteration < noCorrections.  Return limit = iteration - noCorrections if iteration >= noCorrections.
	limit = returnLimit(kIteration, noCorrections);

	// First main for loop.
	counter = 0;
	for (integer i = kIteration - 1; i >= limit; --i)
	{
		correctionNo = correctionHistory[i];

		// Evaluate rho_i = 1.0 / sdoty.
		evaluateAndStoreRho(rhoValues, correctionNo, components, noComponents, sVectors, yVectors);

		// Evaluate alpha_i = rho_i * (sdotq).
		evaluateAndStoreAlpha(alphaValues, rhoValues, counter, qVector, correctionNo, components, noComponents, sVectors);

		// Update qVector = qVector - alpha_i * y.
		updateQVector(qVector, alphaValues, counter, correctionNo, components, noComponents, yVectors);

		++counter;
	}

	correctionNo = correctionHistory[kIteration - 1];

	// Return H0 = sdoty / ydoty.
	scaling[0] = returnH0Factor(correctionNo, components, noComponents, sVectors, yVectors);

	// Return restricted H0 between 0.001 and 1000 so long as positive.
	scaling[0] = returnRestrictedH0Factor(scaling[0]);

	// Set rVector = H0 * qVector.
	setRVector(rVector, scaling[0], qVector, components, noComponents);

	// Second main for loop.
	counter = counter - 1;
	for (integer i = limit; i <= kIteration - 1; ++i)
	{
		correctionNo = correctionHistory[i];

		// Return betaVariable = rho_i * (ydotr).
		betaVariable = returnBetaVariable(rhoValues, counter, rVector, components, noComponents, correctionNo, yVectors);

		// Update rVector = rVector + s * (alpha_i - betaVariable).
		updateRVector(rVector, components, noComponents, correctionNo, sVectors, alphaValues, counter, betaVariable);

		--counter;
	}
}

// Set qVector = gradient.
void LASO::RecursionLBFGS::setQVector(vector &qVector, vector &gradient,
	vectorinteger &components, integer noComponents)
{
	for (integer i = 0; i < noComponents; ++i)
	{
		qVector[components[i]] = gradient[components[i]];
	}
}

// Return limit = 0 if iteration < noCorrections.  Return limit = iteration - noCorrections if iteration >= noCorrections.
integer LASO::RecursionLBFGS::returnLimit(integer currentIteration, integer noCorrections)
{
	integer limit = 0;
	if (currentIteration < noCorrections)
	{
		limit = 0;
	}
	else
	{
		limit = currentIteration - noCorrections;
	}
	return limit;
}

// Evaluate rho_i = 1.0 / sdoty.
void LASO::RecursionLBFGS::evaluateAndStoreRho(vector &rho, integer correctionNo,
	vectorinteger &components, integer noComponents, matrix &sVectors, matrix &yVectors)
{
	doublereal sdoty = 0.0;
	for (integer j = 0; j < noComponents; ++j)
	{
		sdoty += sVectors[correctionNo][components[j]] * yVectors[correctionNo][components[j]];
	}
	rho.push_back(1.0 / sdoty);
}

// Evaluate alpha_i = rho_i * (sdotq). 
void LASO::RecursionLBFGS::evaluateAndStoreAlpha(vector &alpha, vector &rho,
	integer counter, vector &qVector, integer correctionNo,
	vectorinteger &components, integer noComponents, matrix &sVectors)
{
	doublereal sdotq = 0.0;
	for (integer j = 0; j < noComponents; ++j)
	{
		sdotq += sVectors[correctionNo][components[j]] * qVector[components[j]];
	}
	alpha.push_back(rho[counter] * sdotq);
}

// Update qVector = qVector - alpha_i * y.
void LASO::RecursionLBFGS::updateQVector(vector &qVector, vector &alpha,
	integer counter, integer correctionNo, vectorinteger &components, integer noComponents,
	matrix &yVectors)
{
	for (integer j = 0; j < noComponents; ++j)
	{
		qVector[components[j]] -= alpha[counter] * yVectors[correctionNo][components[j]];
	}
}

// Return H0 = sdoty / ydoty.
doublereal LASO::RecursionLBFGS::returnH0Factor(integer correctionNo, vectorinteger &components,
	integer noComponents, matrix &sVectors, matrix &yVectors)
{
	doublereal sdoty = 0.0;
	doublereal ydoty = 0.0;
	for (integer i = 0; i < noComponents; ++i)
	{
		sdoty += sVectors[correctionNo][components[i]] * yVectors[correctionNo][components[i]];
		ydoty += yVectors[correctionNo][components[i]] * yVectors[correctionNo][components[i]];
	}
	return sdoty / ydoty;
}

// Return restricted H0 between 0.001 and 1000 so long as positive.
doublereal LASO::RecursionLBFGS::returnRestrictedH0Factor(doublereal H0)
{
	if (0.0 <= H0 && H0 <= 0.001)
	{
		H0 = 0.001;
	}
	else if (1000.0 <= H0)
	{
		H0 = 1000.0;
	}
	return H0;
}

// Set rVector = H0 * qVector.
void LASO::RecursionLBFGS::setRVector(vector &rVector, doublereal H0,
	vector &qVector, vectorinteger &components, integer noComponents)
{
	for (integer i = 0; i < noComponents; ++i)
	{
		rVector[components[i]] = H0 * qVector[components[i]];
	}
}

// Return betaVariable = rho_i * (ydotr).
doublereal LASO::RecursionLBFGS::returnBetaVariable(vector &rho, integer counter,
	vector &rVector, vectorinteger &components, integer noComponents,
	integer correctionNo, matrix &yVectors)
{
	doublereal ydotr = 0.0;
	for (integer j = 0; j < noComponents; ++j)
	{
		ydotr += yVectors[correctionNo][components[j]] * rVector[components[j]];
	}
	return rho[counter] * ydotr;
}

// Update rVector = rVector + s * (alpha_i - betaVariable).
void LASO::RecursionLBFGS::updateRVector(vector &rVector, vectorinteger &components,
	integer noComponents, integer correctionNo, matrix &sVectors, vector &alpha,
	integer counter, doublereal betaVariable)
{
	for (integer j = 0; j < noComponents; ++j)
	{
		rVector[components[j]] += sVectors[correctionNo][components[j]] * (alpha[counter] - betaVariable);
	}
}