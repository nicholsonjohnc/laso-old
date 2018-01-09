#include "HS035.h"

bool LASO::HS035::objFun(double *x, double *fObj)
{

	fObj[0] = 9.0 - 8.0 * x[0] - 6.0 * x[1] - 4.0 * x[2] + 2.0 * x[0] * x[0] + 2.0 * x[1] * x[1] + x[2] * x[2] + 2.0 * x[0] * x[1] + 2.0 * x[0] * x[2];

	return true;

}

bool LASO::HS035::objGrad(double *x, double *gObj)
{

	gObj[0] = -8.0 + 4.0 * x[0] + 2.0 * x[1] + 2.0 * x[2];
	gObj[1] = -6.0 + 4.0 * x[1] + 2.0 * x[0];
	gObj[2] = -4.0 + 2.0 * x[2] + 2.0 * x[0];

	return true;

}

bool LASO::HS035::conFun(double *x, double *fCon)
{

	fCon[0] = 3.0 - x[0] - x[1] - 2.0 * x[2];

	return true;

}

bool LASO::HS035::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 1 + 0] = -1.0;			// Derivative of the first constraint wrt variable 1.
	jCon[1 * 1 + 0] = -1.0;			// Derivative of the first constraint wrt variable 2.
	jCon[2 * 1 + 0] = -2.0;			// Derivative of the first constraint wrt variable 3.

	return true;

}