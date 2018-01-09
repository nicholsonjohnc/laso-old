#include "HS015.h"

bool LASO::HS015::objFun(double *x, double *fObj)
{

	fObj[0] = 100.0 * pow(x[1] - pow(x[0], 2.0), 2.0) + pow(1.0 - x[0], 2.0);

	return true;

}

bool LASO::HS015::objGrad(double *x, double *gObj)
{

	gObj[0] = -400.0 * x[0] * (x[1] - pow(x[0], 2.0)) + 2.0 * x[0] - 2.0;
	gObj[1] = -200.0 * pow(x[0], 2.0) + 200.0 * x[1];

	return true;

}

bool LASO::HS015::conFun(double *x, double *fCon)
{

	fCon[0] = x[0] * x[1] - 1.0;
	fCon[1] = x[0] + pow(x[1], 2.0);

	//fCon[2] = x[0];

	return true;

}

bool LASO::HS015::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 2 + 0] = x[1];			// Derivative of the first constraint wrt variable 1.
	jCon[1 * 2 + 0] = x[0];			// Derivative of the first constraint wrt variable 2.

	jCon[0 * 2 + 1] = 1.0;			// Derivative of the second constraint wrt variable 1.
	jCon[1 * 2 + 1] = 2.0 * x[1];	// Derivative of the second constraint wrt variable 2.

	//jCon[2][0] = 1.0;			// Derivative of the third constraint wrt variable 1.
	//jCon[2][1] = 0.0;			// Derivative of the third constraint wrt variable 2.

	return true;

}