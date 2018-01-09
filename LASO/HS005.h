#ifndef LASO_HS005
#define LASO_HS005

#include "Common.h"
#include "IFunctions.h"

namespace LASO
{
	// Declare HS005 class.
	class HS005 : public IFunctions
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