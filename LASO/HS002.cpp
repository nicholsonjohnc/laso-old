#include "HS002.h"
#include <cmath>

bool LASO::HS002::objFun(double *x, double *fObj)
{

	fObj[0] = 100.0 * pow((x[1] - x[0] * x[0]), 2.0) + pow((1.0 - x[0]), 2.0);

	return true;

}

bool LASO::HS002::objGrad(double *x, double *gObj)
{

	gObj[0] = 400.0 * pow(x[0], 3.0) - 400.0 * x[0] * x[1] + 2.0 * x[0] - 2.0;
	gObj[1] = -200.0 * x[0] * x[0] + 200.0 * x[1];

	return true;

}

bool LASO::HS002::conFun(double *x, double *fCon)
{

	return true;

}

bool LASO::HS002::conJac(double *x, double *jCon)
{

	return true;

}