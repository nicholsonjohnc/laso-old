#include "FunctionsSNOPT.h"

LASO::FunctionsSNOPT::FunctionsSNOPT(funobjType funobj, funconType funcon, long int nVariables, long int mConstraints, double* fObj, double* gObj, double* fCon, double* gCon)
{
	funObj = funobj;
	funCon = funcon;
	n = nVariables;
	ncnln = mConstraints;
	fObjPointer = fObj;
	gObjPointer = gObj;
	fConPointer = fCon;
	gConPointer = gCon;
}

bool LASO::FunctionsSNOPT::objFun(double *x, double *fObj)
{

	long int mode = 0;
	long int nState = 1;

	funObj(&mode, &n, x, fObj, gObjPointer, &nState);

	return true;
}

bool LASO::FunctionsSNOPT::objGrad(double *x, double *gObj)
{

	long int mode = 1;
	long int nState = 1;

	funObj(&mode, &n, x, fObjPointer, gObj, &nState);

	return true;
}

bool LASO::FunctionsSNOPT::conFun(double *x, double *fCon)
{

	long int mode = 0;
	long int needc = 1;
	long int nState = 1;

	funCon(&mode, &ncnln, &n, &ncnln, &needc, x, fCon, gConPointer, &nState);

	return true;

}

bool LASO::FunctionsSNOPT::conJac(double *x, double *jCon)
{

	long int mode = 1;
	long int needc = 1;
	long int nState = 1;

	funCon(&mode, &ncnln, &n, &ncnln, &needc, x, fConPointer, jCon, &nState);

	return true;

}