#pragma once

#ifndef LASO_FUNCTIONSLM
#define LASO_FUNCTIONSLM

#include "Common.h"
#include "IFunctions.h"

namespace LASO
{
	// Declare FunctionsLM class. 
	class FunctionsLM : public IFunctions
	{

	public:

		// Declare public attributes.
		matrix &AMatrix;							// A matrix.
		vector &aVector;							// a vector.
		long int n;
		long int ncnln;

		FunctionsLM(matrix &A, vector &a, long int nVariables, long int mConstraints);			// Default constructor.
		bool objFun(double *x, double *fObj);		// Function that evaluates objective.
		bool objGrad(double *x, double *gObj);		// Function that evaluates objective gradient.
		bool conFun(double *x, double *fCon);		// Function that evaluates constraints.
		bool conJac(double *x, double *jCon);		// Function that evaluates constraint Jacobian.

	private:

		// Declare private attributes.

	};

}

#endif
