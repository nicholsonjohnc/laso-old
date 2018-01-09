#pragma once

#ifndef LASO_HS071
#define LASO_HS071

#include "Common.h"
#include "IFunctions.h"

namespace LASO
{
	// Declare HS071 class. 
	class HS071 : public IFunctions
	{

	public:

		// Declare public attributes.
		bool objFun(double *x, double *fObj);		// Function that evaluates objective.
		bool objGrad(double *x, double *gObj);		// Function that evaluates objective gradient.
		bool conFun(double *x, double *fCon);		// Function that evaluates constraints.
		bool conJac(double *x, double *jCon);		// Function that evaluates constraint Jacobian.

	private:

		// Declare private attributes.

	};

}

#endif