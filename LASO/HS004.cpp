#include "HS004.h"
#include <cmath>

bool LASO::HS004::objFun(double *x, double *fObj)
{

	fObj[0] = 1.0 / 3.0 * pow((x[0] + 1.0), 3.0) + x[1];

	return true;

}

bool LASO::HS004::objGrad(double *x, double *gObj)
{

	gObj[0] = pow((x[0] + 1.0), 2.0);
	gObj[1] = 1.0;

	return true;

}

bool LASO::HS004::conFun(double *x, double *fCon)
{

	return true;

}

bool LASO::HS004::conJac(double *x, double *jCon)
{

	return true;

}