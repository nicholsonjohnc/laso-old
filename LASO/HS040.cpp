#include "HS040.h"

bool LASO::HS040::objFun(double *x, double *fObj)
{

	fObj[0] = -x[0] * x[1] * x[2] * x[3];

	return true;

}

bool LASO::HS040::objGrad(double *x, double *gObj)
{

	gObj[0] = -x[1] * x[2] * x[3];
	gObj[1] = -x[0] * x[2] * x[3];
	gObj[2] = -x[0] * x[1] * x[3];
	gObj[3] = -x[0] * x[1] * x[2];

	return true;

}

bool LASO::HS040::conFun(double *x, double *fCon)
{

	fCon[0] = pow(x[0], 3.0) + x[1] * x[1] - 1.0;
	fCon[1] = x[0] * x[0] * x[3] - x[2];
	fCon[2] = x[3] * x[3] - x[1];

	return true;

}

bool LASO::HS040::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 3 + 0] = 3.0 * x[0] * x[0];		// Derivative of the first constraint wrt variable 1.
	jCon[1 * 3 + 0] = 2.0 * x[1];			// Derivative of the first constraint wrt variable 2.
	jCon[2 * 3 + 0] = 0.0;					// Derivative of the first constraint wrt variable 3.
	jCon[3 * 3 + 0] = 0.0;					// Derivative of the first constraint wrt variable 4.

	jCon[0 * 3 + 1] = 2.0 * x[0] * x[3];		// Derivative of the second constraint wrt variable 1.
	jCon[1 * 3 + 1] = 0.0;					// Derivative of the second constraint wrt variable 2.
	jCon[2 * 3 + 1] = -1.0;					// Derivative of the second constraint wrt variable 3.
	jCon[3 * 3 + 1] = x[0] * x[0];			// Derivative of the second constraint wrt variable 4.

	jCon[0 * 3 + 2] = 0.0;					// Derivative of the third constraint wrt variable 1.
	jCon[1 * 3 + 2] = -1.0;					// Derivative of the third constraint wrt variable 2.
	jCon[2 * 3 + 2] = 0.0;					// Derivative of the third constraint wrt variable 3.
	jCon[3 * 3 + 2] = 2.0 * x[3];			// Derivative of the third constraint wrt variable 4.

	return true;

}