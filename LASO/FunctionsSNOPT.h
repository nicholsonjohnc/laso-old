#pragma once

#ifndef LASO_FUNCTIONSSNOPT
#define LASO_FUNCTIONSSNOPT

#include "Common.h"
#include "IFunctions.h"

typedef int(*funobjType)(long int *mode, long int *n, double *x, double *fObj, double *gObj, long int *nState);
typedef int(*funconType)(long int *mode, long int *ncnln, long int *n, long int *ldg, long int *needc, double *x, double *fCon, double *gCon, long int *nState);

namespace LASO
{
	// Declare FunctionsSNOPT class. 
	class FunctionsSNOPT : public IFunctions
	{

	public:

		// Declare public attributes.
		funobjType funObj;
		funconType funCon;
		long int n;
		long int ncnln;
		double* fObjPointer;
		double* gObjPointer;
		double* fConPointer;
		double* gConPointer;

		FunctionsSNOPT(funobjType funobj, funconType funcon, long int nVariables, long int mConstraints, double* fObj, double* gObj, double* fCon, double* gCon);		// Default constructor.
		bool objFun(double *x, double *fObj);		// Function that evaluates objective.
		bool objGrad(double *x, double *gObj);		// Function that evaluates objective gradient.
		bool conFun(double *x, double *fCon);		// Function that evaluates constraints.
		bool conJac(double *x, double *jCon);		// Function that evaluates constraint Jacobian.

	private:

		// Declare private attributes.

	};

}

#endif