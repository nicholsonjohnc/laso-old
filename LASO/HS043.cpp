#include "HS043.h"

bool LASO::HS043::objFun(double *x, double *fObj)
{

	fObj[0] = x[0] * x[0] + x[1] * x[1] + 2.0 * x[2] * x[2] + x[3] * x[3] - 5.0 * x[0] - 5.0 * x[1] - 21.0 * x[2] + 7.0 * x[3];

	return true;

}

bool LASO::HS043::objGrad(double *x, double *gObj)
{

	gObj[0] = 2.0 * x[0] - 5.0;
	gObj[1] = 2.0 * x[1] - 5.0;
	gObj[2] = 4.0 * x[2] - 21.0;
	gObj[3] = 2.0 * x[3] + 7.0;

	return true;

}

bool LASO::HS043::conFun(double *x, double *fCon)
{

	fCon[0] = 8.0 - x[0] * x[0] - x[1] * x[1] - x[2] * x[2] - x[3] * x[3] - x[0] + x[1] - x[2] + x[3];
	fCon[1] = 10.0 - x[0] * x[0] - 2.0 * x[1] * x[1] - x[2] * x[2] - 2.0 * x[3] * x[3] + x[0] + x[3];
	fCon[2] = 5.0 - 2.0 * x[0] * x[0] - x[1] * x[1] - x[2] * x[2] - 2.0 * x[0] + x[1] + x[3];

	return true;

}

bool LASO::HS043::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 3 + 0] = -2.0 * x[0] - 1.0;		// derivative of the first constraint wrt variable 1
	jCon[1 * 3 + 0] = -2.0 * x[1] + 1.0;		// derivative of the first constraint wrt variable 2
	jCon[2 * 3 + 0] = -2.0 * x[2] - 1.0;		// derivative of the first constraint wrt variable 3
	jCon[3 * 3 + 0] = -2.0 * x[3] + 1.0;		// derivative of the first constraint wrt variable 4

	jCon[0 * 3 + 1] = -2.0 * x[0] + 1.0;		// derivative of the second constraint wrt variable 1
	jCon[1 * 3 + 1] = -4.0 * x[1];				// derivative of the second constraint wrt variable 2
	jCon[2 * 3 + 1] = -2.0 * x[2];				// derivative of the second constraint wrt variable 3
	jCon[3 * 3 + 1] = -4.0 * x[3] + 1.0;		// derivative of the second constraint wrt variable 4

	jCon[0 * 3 + 2] = -4.0 * x[0] - 2.0;		// derivative of the third constraint wrt variable 1
	jCon[1 * 3 + 2] = -2.0 * x[1] + 1.0;		// derivative of the third constraint wrt variable 2
	jCon[2 * 3 + 2] = -2.0 * x[2];				// derivative of the third constraint wrt variable 3
	jCon[3 * 3 + 2] = 1.0;						// derivative of the third constraint wrt variable 4

	return true;

}