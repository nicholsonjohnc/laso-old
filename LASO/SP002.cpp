#include "SP002.h"
#include <cmath>

LASO::SP002::SP002(long int nVariables, long int mConstraints)
{
	n = nVariables;
	ncnln = mConstraints;
}

bool LASO::SP002::objFun(double *x, double *fObj)
{

	fObj[0] = 0;
	for (integer i = 0; i < n - 1; ++i)
	{
		fObj[0] += pow((1.0 - x[i]), 2.0) + 100.0 * pow((-pow(x[i], 2.0) + x[i + 1]), 2.0);
	}

	return true;
}

bool LASO::SP002::objGrad(double *x, double *gObj)
{

	gObj[0] = 2.0 * (-1.0 + x[0] + 200.0 * pow(x[0], 3.0) - 200.0 * x[0] * x[1]);
	for (integer i = 1; i < n - 1; ++i)
	{
		gObj[i] = (-2.0 - 200.0 * pow(x[i - 1], 2.0) + 400.0 * pow(x[i], 3.0) + x[i] * (202.0 - 400.0 * x[i + 1]));
	}
	gObj[n - 1] = 200.0 * (x[n - 1] - pow(x[n - 2], 2.0));

	return true;
}

bool LASO::SP002::conFun(double *x, double *fCon)
{

	return true;

}

bool LASO::SP002::conJac(double *x, double *jCon)
{

	return true;

}