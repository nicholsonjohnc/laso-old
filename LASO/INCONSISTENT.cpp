#include "INCONSISTENT.h"

bool LASO::INCONSISTENT::objFun(double *x, double *fObj)
{

	fObj[0] = -x[0];

	return true;

}

bool LASO::INCONSISTENT::objGrad(double *x, double *gObj)
{

	gObj[0] = -1.0;

	return true;

}

bool LASO::INCONSISTENT::conFun(double *x, double *fCon)
{

	fCon[0] = x[0];
	fCon[1] = x[0] * x[0];

	//fCon[0] = x[0] * x[0];

	return true;

}

bool LASO::INCONSISTENT::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 2 + 0] = 1.0;				// Derivative of the first constraint wrt variable 1.

	jCon[0 * 2 + 1] = 2.0 * x[0];		// Derivative of the second constraint wrt variable 1.

	return true;

}