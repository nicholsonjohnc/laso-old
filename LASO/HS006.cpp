#include "HS006.h"

bool LASO::HS006::objFun(double *x, double *fObj)
{

	fObj[0] = pow((1.0 - x[0]), 2.0);

	return true;

}

bool LASO::HS006::objGrad(double *x, double *gObj)
{

	gObj[0] = 2.0 * (1.0 - x[0]) * (-1.0);
	gObj[1] = 0.0;

	return true;

}

bool LASO::HS006::conFun(double *x, double *fCon)
{

	fCon[0] = 10.0 * (x[1] - x[0] * x[0]);

	return true;

}

bool LASO::HS006::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional vector.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 1 + 0] = -20.0 * x[0];	// Derivative of the first constraint wrt variable 1.
	jCon[1 * 1 + 0] = 10.0;	// Derivative of the first constraint wrt variable 2.

	return true;

}