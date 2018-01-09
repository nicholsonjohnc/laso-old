#include "HS039.h"

bool LASO::HS039::objFun(double *x, double *fObj)
{

	fObj[0] = -x[0];

	return true;

}

bool LASO::HS039::objGrad(double *x, double *gObj)
{

	gObj[0] = -1.0;
	gObj[1] = 0.0;
	gObj[2] = 0.0;
	gObj[3] = 0.0;

	return true;

}

bool LASO::HS039::conFun(double *x, double *fCon)
{

	fCon[0] = x[1] - x[0] * x[0] * x[0] - x[2] * x[2];
	fCon[1] = x[0] * x[0] - x[1] - x[3] * x[3];

	return true;

}

bool LASO::HS039::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 2 + 0] = -3.0 * x[0] * x[0];	// Derivative of the first constraint wrt variable 1.
	jCon[1 * 2 + 0] = 1.0;					// Derivative of the first constraint wrt variable 2.
	jCon[2 * 2 + 0] = -2.0 * x[2];			// Derivative of the first constraint wrt variable 3.
	jCon[3 * 2 + 0] = 0.0;					// Derivative of the first constraint wrt variable 4.

	jCon[0 * 2 + 1] = 2.0 * x[0];			// Derivative of the second constraint wrt variable 1.
	jCon[1 * 2 + 1] = -1.0;					// Derivative of the second constraint wrt variable 2.
	jCon[2 * 2 + 1] = 0.0;					// Derivative of the second constraint wrt variable 3.
	jCon[3 * 2 + 1] = -2.0 * x[3];			// Derivative of the second constraint wrt variable 4.

	return true;

}