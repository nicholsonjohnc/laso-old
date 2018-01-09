#include "HS010.h"

bool LASO::HS010::objFun(double *x, double *fObj)
{

	fObj[0] = x[0] - x[1];

	return true;

}

bool LASO::HS010::objGrad(double *x, double *gObj)
{

	gObj[0] = 1.0;
	gObj[1] = -1.0;

	return true;

}

bool LASO::HS010::conFun(double *x, double *fCon)
{

	fCon[0] = -3.0 * x[0] * x[0] + 2.0 * x[0] * x[1] - x[1] * x[1] + 1;

	return true;

}

bool LASO::HS010::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 1 + 0] = -6.0 * x[0] + 2.0 * x[1];	// Derivative of the first constraint wrt variable 1.
	jCon[1 * 1 + 0] = 2.0 * x[0] - 2.0 * x[1];	// Derivative of the first constraint wrt variable 2.

	return true;

}