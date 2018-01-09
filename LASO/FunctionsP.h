#pragma once

#ifndef LASO_FUNCTIONSP
#define LASO_FUNCTIONSP

#include "Common.h"
#include "IFunctions.h"

namespace LASO
{
	// Declare FunctionsP class. 
	class FunctionsP : public IFunctions
	{

	public:

		// Declare public attributes.
		vector &CMatrix;							// C vector.
		vector &cVector;							// c vector.
		long int n;
		long int ncnln;

		FunctionsP(vector &C, vector &c, long int nVariables, long int mConstraints);			// Default constructor.
		bool objFun(double *x, double *fObj);		// Function that evaluates objective.
		bool objGrad(double *x, double *gObj);		// Function that evaluates objective gradient.
		bool conFun(double *x, double *fCon);		// Function that evaluates constraints.
		bool conJac(double *x, double *jCon);		// Function that evaluates constraint Jacobian.

	private:

		// Declare private attributes.

	};

}

#endif