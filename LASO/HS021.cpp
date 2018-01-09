#include "HS021.h"

bool LASO::HS021::objFun(double *x, double *fObj)
{

	fObj[0] = 0.01 * x[0] * x[0] + x[1] * x[1] - 100.0;

	return true;

}

bool LASO::HS021::objGrad(double *x, double *gObj)
{

	gObj[0] = 0.02 * x[0];
	gObj[1] = 2.0 * x[1];

	return true;

}

bool LASO::HS021::conFun(double *x, double *fCon)
{

	fCon[0] = 10.0 * x[0] - x[1] - 10.0;

	return true;

}

bool LASO::HS021::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 1 + 0] = 10.0;				// Derivative of the first constraint wrt variable 1.
	jCon[1 * 1 + 0] = -1.0;				// Derivative of the first constraint wrt variable 2.

	return true;

}