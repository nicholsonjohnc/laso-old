//#ifndef LASO_ROCKET
//#define LASO_ROCKET
//
//#include "Common.h"
//#include "IFunctions.h"
//
//namespace LASO
//{
//	// Declare ROBOT class.
//	class ROBOT : public IFunctions
//	{
//
//	public:
//
//		// Declare public attributes.
//		long int n;
//		long int ncnln;
//		doublereal l;
//
//		ROBOT(long int nVariables, long int mConstraints, doublereal L);			// Default constructor.
//		bool objFun(double *x, double *fObj);		// Function that evaluates objective.
//		bool objGrad(double *x, double *gObj);		// Function that evaluates objective gradient.
//		bool conFun(double *x, double *fCon);		// Function that evaluates constraints.
//		bool conJac(double *x, double *jCon);		// Function that evaluates constraint Jacobian.
//
//	private:
//
//		// Declare private attributes.
//
//	};
//
//}
//
//#endif