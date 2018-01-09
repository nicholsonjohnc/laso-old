#include "HS055.h"

bool LASO::HS055::objFun(double *x, double *fObj)
{

	fObj[0] = x[0] + 2.0 * x[1] + 4.0 * x[4] + exp(x[0] * x[3]);

	return true;

}

bool LASO::HS055::objGrad(double *x, double *gObj)
{

	gObj[0] = 1.0 + x[3] * exp(x[0] * x[3]);
	gObj[1] = 2.0;
	gObj[2] = 0.0;
	gObj[3] = x[0] * exp(x[0] * x[3]);
	gObj[4] = 4.0;
	gObj[5] = 0.0;

	return true;

}

bool LASO::HS055::conFun(double *x, double *fCon)
{

	fCon[0] = x[0] + 2.0 * x[1] + 5.0 * x[4] - 6.0;
	fCon[1] = x[0] + x[1] + x[2] - 3.0;
	fCon[2] = x[3] + x[4] + x[5] - 2.0;
	fCon[3] = x[0] + x[3] - 1.0;
	fCon[4] = x[1] + x[4] - 2.0;
	fCon[5] = x[2] + x[5] - 2.0;

	return true;

}

bool LASO::HS055::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 6 + 0] = 1.0;					// Derivative of the first constraint wrt variable 1.
	jCon[1 * 6 + 0] = 2.0;					// Derivative of the first constraint wrt variable 2.
	jCon[2 * 6 + 0] = 0.0;					// Derivative of the first constraint wrt variable 3.
	jCon[3 * 6 + 0] = 0.0;					// Derivative of the first constraint wrt variable 4.
	jCon[4 * 6 + 0] = 5.0;					// Derivative of the first constraint wrt variable 5.
	jCon[5 * 6 + 0] = 0.0;					// Derivative of the first constraint wrt variable 6.

	jCon[0 * 6 + 1] = 1.0;					// Derivative of the second constraint wrt variable 1.
	jCon[1 * 6 + 1] = 1.0;					// Derivative of the second constraint wrt variable 2.
	jCon[2 * 6 + 1] = 1.0;					// Derivative of the second constraint wrt variable 3.
	jCon[3 * 6 + 1] = 0.0;					// Derivative of the second constraint wrt variable 4.
	jCon[4 * 6 + 1] = 0.0;					// Derivative of the second constraint wrt variable 5.
	jCon[5 * 6 + 1] = 0.0;					// Derivative of the second constraint wrt variable 6.

	jCon[0 * 6 + 2] = 0.0;					// Derivative of the third constraint wrt variable 1.
	jCon[1 * 6 + 2] = 0.0;					// Derivative of the third constraint wrt variable 2.
	jCon[2 * 6 + 2] = 0.0;					// Derivative of the third constraint wrt variable 3.
	jCon[3 * 6 + 2] = 1.0;					// Derivative of the third constraint wrt variable 4.
	jCon[4 * 6 + 2] = 1.0;					// Derivative of the third constraint wrt variable 5.
	jCon[5 * 6 + 2] = 1.0;					// Derivative of the third constraint wrt variable 6.

	jCon[0 * 6 + 3] = 1.0;					// Derivative of the fourth constraint wrt variable 1.
	jCon[1 * 6 + 3] = 0.0;					// Derivative of the fourth constraint wrt variable 2.
	jCon[2 * 6 + 3] = 0.0;					// Derivative of the fourth constraint wrt variable 3.
	jCon[3 * 6 + 3] = 1.0;					// Derivative of the fourth constraint wrt variable 4.
	jCon[4 * 6 + 3] = 0.0;					// Derivative of the fourth constraint wrt variable 5.
	jCon[5 * 6 + 3] = 0.0;					// Derivative of the fourth constraint wrt variable 6.

	jCon[0 * 6 + 4] = 0.0;					// Derivative of the fifth constraint wrt variable 1.
	jCon[1 * 6 + 4] = 1.0;					// Derivative of the fifth constraint wrt variable 2.
	jCon[2 * 6 + 4] = 0.0;					// Derivative of the fifth constraint wrt variable 3.
	jCon[3 * 6 + 4] = 0.0;					// Derivative of the fifth constraint wrt variable 4.
	jCon[4 * 6 + 4] = 1.0;					// Derivative of the fifth constraint wrt variable 5.
	jCon[5 * 6 + 4] = 0.0;					// Derivative of the fifth constraint wrt variable 6.

	jCon[0 * 6 + 5] = 0.0;					// Derivative of the sixth constraint wrt variable 1.
	jCon[1 * 6 + 5] = 0.0;					// Derivative of the sixth constraint wrt variable 2.
	jCon[2 * 6 + 5] = 1.0;					// Derivative of the sixth constraint wrt variable 3.
	jCon[3 * 6 + 5] = 0.0;					// Derivative of the sixth constraint wrt variable 4.
	jCon[4 * 6 + 5] = 0.0;					// Derivative of the sixth constraint wrt variable 5.
	jCon[5 * 6 + 5] = 1.0;					// Derivative of the sixth constraint wrt variable 6.

	return true;

}