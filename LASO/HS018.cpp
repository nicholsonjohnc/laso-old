#include "HS018.h"

bool LASO::HS018::objFun(double *x, double *fObj)
{

	fObj[0] = 0.01 * x[0] * x[0] + x[1] * x[1];

	return true;

}

bool LASO::HS018::objGrad(double *x, double *gObj)
{

	gObj[0] = 0.02 * x[0];
	gObj[1] = 2.0 * x[1];

	return true;

}

bool LASO::HS018::conFun(double *x, double *fCon)
{

	fCon[0] = x[0] * x[1] - 25.0;
	fCon[1] = x[0] * x[0] + x[1] * x[1] - 25.0;

	return true;

}

bool LASO::HS018::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 2 + 0] = x[1];				// Derivative of the first constraint wrt variable 1.
	jCon[1 * 2 + 0] = x[0];				// Derivative of the first constraint wrt variable 2.

	jCon[0 * 2 + 1] = 2.0 * x[0];				// Derivative of the second constraint wrt variable 1.
	jCon[1 * 2 + 1] = 2.0 * x[1];				// Derivative of the second constraint wrt variable 2.

	return true;

}