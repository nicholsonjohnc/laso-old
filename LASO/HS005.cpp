#include "HS005.h"
#include <cmath>

bool LASO::HS005::objFun(double *x, double *fObj)
{

	fObj[0] = sin(x[0] + x[1]) + pow((x[0] - x[1]), 2.0) - 1.5 * x[0] + 2.5 * x[1] + 1.0;

	return true;

}

bool LASO::HS005::objGrad(double *x, double *gObj)
{

	gObj[0] = cos(x[0] + x[1]) + 2.0 * (x[0] - x[1]) - 1.5;
	gObj[1] = cos(x[0] + x[1]) - 2.0 * (x[0] - x[1]) + 2.5;

	return true;

}

bool LASO::HS005::conFun(double *x, double *fCon)
{

	return true;

}

bool LASO::HS005::conJac(double *x, double *jCon)
{

	return true;

}