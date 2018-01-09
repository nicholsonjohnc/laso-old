#include "SPRING.h"

bool LASO::SPRING::objFun(double *x, double *fObj)
{

	fObj[0] = pow(x[0], 2.0) * x[1] * (2.0 + x[2]);

	return true;

}

bool LASO::SPRING::objGrad(double *x, double *gObj)
{

	gObj[0] = 2.0 * x[0] * x[1] * (2.0 + x[2]);
	gObj[1] = pow(x[0], 2.0) * (2.0 + x[2]);
	gObj[2] = pow(x[0], 2.0) * x[1];

	return true;

}

bool LASO::SPRING::conFun(double *x, double *fCon)
{

	fCon[0] = 1.0 - (pow(x[1], 3.0)*x[2]) / (71875.0 * pow(x[0], 4.0));
	fCon[1] = -1.0 + 123.0 / (628300.0 * pow(x[0], 2.0)) + (x[1] * (-x[0] + 4.0 * x[1])) / (12566.0 * pow(x[0], 3.0)*(-x[0] + x[1]));
	fCon[2] = 1.0 - (140.54*x[0]) / (pow(x[1], 2.0)*x[2]);
	fCon[3] = -1.0 + (2.0 * (x[0] + x[1])) / 3.0;

	return true;

}

bool LASO::SPRING::conJac(double *x, double *jCon)
{

	// Note: Jacobian is stored in single dimensional array.
	// Numbering system is as follows:
	// [design variable number * total number of constraints + constraint number]
	jCon[0 * 4 + 0] = (4.0 * pow(x[1], 3.0)*x[2]) / (71875.0 * pow(x[0], 5.0));
	jCon[1 * 4 + 0] = -((3.0 * pow(x[1], 2.0)*x[2]) / (71875.0 * pow(x[0], 4.0)));
	jCon[2 * 4 + 0] = -(pow(x[1], 3.0) / (71875.0 * pow(x[0], 4.0)));

	jCon[0 * 4 + 1] = -((3.0 * (41.0 * pow(x[0], 3.0) - 57.0 * pow(x[0], 2.0)*x[1] - 109.0 * x[0] * pow(x[1], 2.0) + 100.0 * pow(x[1], 3.0))) / (314150.0 * pow(x[0], 4.0)*pow((x[0] - x[1]), 2.0)));
	jCon[1 * 4 + 1] = (pow(x[0], 2.0) - 8.0 * x[0] * x[1] + 4.0 * pow(x[1], 2.0)) / (12566.0 * pow(x[0], 3.0)*pow((x[0] - x[1]), 2.0));
	jCon[2 * 4 + 1] = 0.0;

	jCon[0 * 4 + 2] = -(140.54 / (pow(x[1], 2.0)*x[2]));
	jCon[1 * 4 + 2] = (281.08*x[0]) / (pow(x[1], 3.0)*x[2]);
	jCon[2 * 4 + 2] = (140.54*x[0]) / (pow(x[1], 2.0)*pow(x[2], 2.0));

	jCon[0 * 4 + 3] = 2.0 / 3.0;
	jCon[1 * 4 + 3] = 2.0 / 3.0;
	jCon[2 * 4 + 3] = 0.0;

	return true;

}