#include "FunctionsP.h"
#include <cmath>

LASO::FunctionsP::FunctionsP(vector &C, vector &c, long int nVariables, long int mConstraints) :
	CMatrix(C),
	cVector(c)
{
	n = nVariables;
	ncnln = mConstraints;
}

bool LASO::FunctionsP::objFun(double *x, double *fObj)
{
	fObj[0] = 0.0;
	doublereal intermediateVariable = 0.0;

	// Handle first n - 1 rows.
	for (integer i = 0; i < n - 1; ++i)
	{
		fObj[0] += pow(2.0 * x[i] + CMatrix[i] * x[n - 1], 2.0);
	}
	// Handle nth row.
	for (integer i = 0; i < n - 1; ++i)
	{
		intermediateVariable += CMatrix[i] * x[i];
	}
	intermediateVariable -= cVector[0];
	fObj[0] += pow(intermediateVariable, 2.0);

	fObj[0] *= 0.5;

	return true;
}

bool LASO::FunctionsP::objGrad(double *x, double *gObj)
{
	doublereal intermediateVariable;

	// Handle first n - 1 rows.
	for (integer i = 0; i < n - 1; ++i)
	{
		gObj[i] = 2.0 * (2.0 * x[i] + CMatrix[i] * x[n - 1]);

		intermediateVariable = 0.0;
		for (integer j = 0; j < n - 1; ++j)
		{
			intermediateVariable += CMatrix[j] * x[j];
		}
		intermediateVariable -= cVector[0];
		gObj[i] += CMatrix[i] * intermediateVariable;
	}

	// Handle nth row.
	gObj[n - 1] = 0.0;
	for (integer i = 0; i < n - 1; ++i)
	{
		gObj[n - 1] += CMatrix[i] * (2.0 * x[i] + CMatrix[i] * x[n - 1]);
	}

	return true;
}

bool LASO::FunctionsP::conFun(double *x, double *fCon)
{

	return true;

}

bool LASO::FunctionsP::conJac(double *x, double *jCon)
{

	return true;

}