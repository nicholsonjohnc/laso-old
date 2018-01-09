#include "SP007.h"
#include <cmath>

bool LASO::SP007::objFun(double *x, double *fObj)
{
	double thetaVar = 0.0;
	
	if( x[0] > 0.0 )
	{
		thetaVar = atan( x[1] / x[0] ) / ( 2.0 * 3.14159265358979323846 );
	}
	else
	{
		thetaVar = ( 3.14159265358979323846 + atan( x[1] / x[0] ) ) / ( 2.0 * 3.14159265358979323846 );
	}
	
	fObj[0] = 100.0 * ( pow( x[2] - 10.0 * thetaVar , 2.0 ) +
		pow( sqrt( pow( x[0] , 2.0 ) + pow( x[1] , 2.0 ) ) - 1.0 , 2.0 ) ) + 
		pow( x[2] , 2.0 );

	return true;
}

bool LASO::SP007::objGrad(double *x, double *gObj)
{
	double thetaVar = 0.0;

	if (x[0] > 0.0)
	{
		thetaVar = atan(x[1] / x[0]) / (2.0 * 3.14159265358979323846);
	}
	else
	{
		thetaVar = (3.14159265358979323846 + atan(x[1] / x[0])) / (2.0 * 3.14159265358979323846);
	}
	
	double dthetaVardx0 = - x[1] /
		(2.0 * 3.14159265358979323846 * pow(x[0], 2.0) * (1.0 + pow(x[1] / x[0], 2.0)));
	
	double dthetaVardx1 = 1.0 /
		(2.0 * 3.14159265358979323846 * x[0] * (1.0 + pow(x[1] / x[0], 2.0)));
	
	gObj[0] = (200.0 * x[0] * (-1.0 + sqrt(pow(x[0], 2.0) + pow(x[1], 2.0)))) / sqrt(pow(x[0], 2.0) + pow(x[1], 2.0)) - 2000.0 * dthetaVardx0 * (x[2] - 10.0 * thetaVar);

	gObj[1] = (200.0 * x[1] * (-1.0 + sqrt(pow(x[0], 2.0) + pow(x[1], 2.0)))) / sqrt(pow(x[0], 2.0) + pow(x[1], 2.0)) - 2000.0 * dthetaVardx1 * (x[2] - 10.0 * thetaVar);

	gObj[2] = 2.0 * x[2] + 200.0 * (x[2] - 10.0 * thetaVar);

	return true;
}

bool LASO::SP007::conFun(double *x, double *fCon)
{

	return true;

}

bool LASO::SP007::conJac(double *x, double *jCon)
{

	return true;

}