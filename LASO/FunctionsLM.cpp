#include "FunctionsLM.h"
#include <cmath>

LASO::FunctionsLM::FunctionsLM(matrix &A, vector &a, long int nVariables, long int mConstraints) :
	AMatrix(A),
	aVector(a)
{
	n = nVariables;
	ncnln = mConstraints;
}

bool LASO::FunctionsLM::objFun(double *x, double *fObj)
{
	doublereal intermediateVariable = 0.0;

	fObj[0] = 0.0;

	for (integer i = 0; i < n; ++i)
	{
		intermediateVariable = 0.0;
		for (integer j = 0; j < n; ++j)
		{
			intermediateVariable += AMatrix[i][j] * x[j];
		}
		intermediateVariable -= aVector[i];

		fObj[0] += 0.5 * pow(intermediateVariable, 2.0);
	}

	return true;
}

bool LASO::FunctionsLM::objGrad(double *x, double *gObj)
{
	doublereal intermediateVariable = 0.0;

	for (integer i = 0; i < n; ++i)
	{
		gObj[i] = 0.0;
	}

	for (integer i = 0; i < n; ++i)
	{
		intermediateVariable = 0.0;
		for (integer j = 0; j < n; ++j)
		{
			intermediateVariable += AMatrix[i][j] * x[j];
		}
		intermediateVariable -= aVector[i];

		for (integer j = 0; j < n; ++j)
		{
			gObj[j] += AMatrix[i][j] * intermediateVariable;
		}
	}

	return true;
}

bool LASO::FunctionsLM::conFun(double *x, double *fCon)
{

	return true;

}

bool LASO::FunctionsLM::conJac(double *x, double *jCon)
{

	return true;

}