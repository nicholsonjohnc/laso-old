#ifndef LASO_SP002
#define LASO_SP002

#include "Common.h"
#include "IFunctions.h"

namespace LASO
{
	// Declare SP002 class.
	class SP002 : public IFunctions
	{

	public:

		// Declare public attributes.
		long int n;
		long int ncnln;

		SP002(long int nVariables, long int mConstraints);		// Default constructor.
		bool objFun(double *x, double *fObj);		// Function that evaluates objective.
		bool objGrad(double *x, double *gObj);		// Function that evaluates objective gradient.
		bool conFun(double *x, double *fCon);		// Function that evaluates constraints.
		bool conJac(double *x, double *jCon);		// Function that evaluates constraint Jacobian.

	private:

		// Declare private attributes.

	};

}

#endif