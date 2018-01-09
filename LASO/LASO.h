#pragma once

#ifdef LASO_EXPORTS
#define LASO_API __declspec(dllexport) 
#else
#define LASO_API __declspec(dllimport) 
#endif

typedef int(*funobjType)(long int *mode, long int *n, double *x, double *fObj, double *gObj, long int *nState);
typedef int(*funconType)(long int *mode, long int *ncnln, long int *n, long int *ldg, long int *needc, double *x, double *fCon, double *gCon, long int *nState);

namespace LASO
{
	// This class is exported from the LASO.dll
	class Engine
	{

	public:

		static LASO_API void solve(long int *n, double *x, funobjType funobj,
			double *lb, double *ub, long int *ncnln, funconType funcon, 
			double* fObj, double* gObj, double* fCon, double* gCon);

	};
}