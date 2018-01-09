#include "HS033.h"

bool LASO::HS033::objFun(double *x, double *fObj)
{

	fObj[0] = (x[0] - 1.0) * (x[0] - 2.0) * (x[0] - 3.0) + x[2];

	return true;

}

bool LASO::HS033::objGrad(double *x, double *gObj)
{

	gObj[0] = (x[0] - 1.0) * (x[0] - 2.0) + (x[0] - 1.0) * (x[0] - 3.0) + (x[0] - 2.0) * (x[0] - 3.0);
	gObj[1] = 0.0;
	gObj[2] = 1.0;

	return true;

}

bool LASO::HS033::conFun(double *x, double *fCon)
{

	fCon[0] = x[2] * x[2] - x[1] * x[1] - x[0] * x[0];
	fCon[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] - 4.0;

	return true;

}

bool LASO::HS033::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 2 + 0] = -2.0 * x[0];			// Derivative of the first constraint wrt variable 1.
	jCon[1 * 2 + 0] = -2.0 * x[1];			// Derivative of the first constraint wrt variable 2.
	jCon[2 * 2 + 0] = 2.0 * x[2];			// Derivative of the first constraint wrt variable 3.

	jCon[0 * 2 + 1] = 2.0 * x[0];			// Derivative of the second constraint wrt variable 1.
	jCon[1 * 2 + 1] = 2.0 * x[1];			// Derivative of the second constraint wrt variable 2.
	jCon[2 * 2 + 1] = 2.0 * x[2];			// Derivative of the second constraint wrt variable 3.

	return true;

}