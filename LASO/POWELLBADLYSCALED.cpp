#include "POWELLBADLYSCALED.h"
#include <cmath>

bool LASO::POWELLBADLYSCALED::objFun(double *x, double *fObj)
{
	fObj[0] = pow((10000.0 * x[0] * x[1] - 1.0), 2.0) + pow((exp(-x[0]) + exp(-x[1]) - 1.0001), 2.0);

	return true;
}

bool LASO::POWELLBADLYSCALED::objGrad(double *x, double *gObj)
{
	gObj[0] = 20000.0 * x[1] * (10000.0 * x[0] * x[1] - 1.0) - 2.0 * exp(-x[0])*(exp(-x[0]) + exp(-x[1]) - 1.0001);
	gObj[1] = 20000.0 * x[0] * (10000.0 * x[0] * x[1] - 1.0) - 2.0 * exp(-x[1])*(exp(-x[0]) + exp(-x[1]) - 1.0001);

	return true;
}

bool LASO::POWELLBADLYSCALED::conFun(double *x, double *fCon)
{

	return true;

}

bool LASO::POWELLBADLYSCALED::conJac(double *x, double *jCon)
{

	return true;

}