#pragma once

#ifndef LASO_IFUNCTIONS
#define LASO_IFUNCTIONS

#include "Common.h"

namespace LASO
{
	// Declare IFunctions interface class. 
	class IFunctions
	{

	public:

		// Declare public attributes.
		virtual bool objFun(double *x, double *fObj) = 0;		// Function that evaluates objective.
		virtual bool objGrad(double *x, double *gObj) = 0;		// Function that evaluates objective gradient.
		virtual bool conFun(double *x, double *fCon) = 0;		// Function that evaluates constraints.
		virtual bool conJac(double *x, double *jCon) = 0;		// Function that evaluates constraint Jacobian.
		virtual ~IFunctions() {};								// Virtual destructor.

	private:

		// Declare private attributes.

	};

}

#endif 