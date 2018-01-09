//#include "ROBOT.h"
//
//LASO::ROBOT::ROBOT(long int nVariables, long int mConstraints, doublereal L)
//{
//	n = nVariables;
//	ncnln = mConstraints;
//	l = L;
//}
//
//bool LASO::ROBOT::objFun(double *x, double *fObj)
//{
//
//	fObj[0] = x[n - 1];
//
//	return true;
//
//}
//
//bool LASO::ROBOT::objGrad(double *x, double *gObj)
//{
//
//	gObj[n - 1] = 1.0;
//
//	return true;
//
//}
//
//bool LASO::ROBOT::conFun(double *x, double *fCon)
//{
//	integer nh = ncnln / 6;
//
//	doublereal step = (x[n - 1] / nh);
//
//	doublereal rho_i = 0.0;
//	doublereal rho_iMinus1 = 0.0;
//	doublereal the_i = 0.0;
//	doublereal the_iMinus1 = 0.0;
//	doublereal phi_i = 0.0;
//	doublereal phi_iMinus1 = 0.0;
//
//	doublereal rho_dot_i = 0.0;
//	doublereal rho_dot_iMinus1 = 0.0;
//	doublereal the_dot_i = 0.0;
//	doublereal the_dot_iMinus1 = 0.0;
//	doublereal phi_dot_i = 0.0;
//	doublereal phi_dot_iMinus1 = 0.0;
//
//	doublereal u_rho_i = 0.0;
//	doublereal u_rho_iMinus1 = 0.0;
//	doublereal u_the_i = 0.0;
//	doublereal u_the_iMinus1 = 0.0;
//	doublereal u_phi_i = 0.0;
//	doublereal u_phi_iMinus1 = 0.0;
//
//	doublereal I_the_i = 0.0;
//	doublereal I_the_iMinus1 = 0.0;
//
//	doublereal I_phi_i = 0.0;
//	doublereal I_phi_iMinus1 = 0.0;
//
//	integer iStart = 1;
//	integer iEnd = nh + 1;
//	for (integer i = iStart; i < iEnd; ++i)
//	{
//		rho_i = x[0 * (nh + 1) + i];
//		rho_iMinus1 = x[0 * (nh + 1) + i - 1];
//		the_i = x[1 * (nh + 1) + i];
//		the_iMinus1 = x[1 * (nh + 1) + i - 1];
//		phi_i = x[2 * (nh + 1) + i];
//		phi_iMinus1 = x[2 * (nh + 1) + i - 1];
//
//		rho_dot_i = x[3 * (nh + 1) + i];
//		rho_dot_iMinus1 = x[3 * (nh + 1) + i - 1];
//		the_dot_i = x[4 * (nh + 1) + i];
//		the_dot_iMinus1 = x[4 * (nh + 1) + i - 1];
//		phi_dot_i = x[5 * (nh + 1) + i];
//		phi_dot_iMinus1 = x[5 * (nh + 1) + i - 1];
//
//		u_rho_i = x[6 * (nh + 1) + i];
//		u_rho_iMinus1 = x[6 * (nh + 1) + i - 1];
//		u_the_i = x[7 * (nh + 1) + i];
//		u_the_iMinus1 = x[7 * (nh + 1) + i - 1];
//		u_phi_i = x[8 * (nh + 1) + i];
//		u_phi_iMinus1 = x[8 * (nh + 1) + i - 1];
//
//		I_the_i = (pow(l - rho_i, 3.0) + pow(rho_i, 3.0)) * pow(sin(phi_i), 2.0) / 3.0;
//		I_the_iMinus1 = (pow(l - rho_iMinus1, 3.0) + pow(rho_iMinus1, 3.0)) * pow(sin(phi_iMinus1), 2.0) / 3.0;
//
//		I_phi_i = (pow(l - rho_i, 3.0) + pow(rho_i, 3)) / 3.0;
//		I_phi_iMinus1 = (pow(l - rho_iMinus1, 3.0) + pow(rho_iMinus1, 3)) / 3.0;
//
//		// rho_eqn
//		fCon[0 * nh + i - 1] = rho_i - (rho_iMinus1 + 0.5*step*(rho_dot_i + rho_dot_iMinus1));
//
//		// the_eqn
//		fCon[1 * nh + i - 1] = the_i - (the_iMinus1 + 0.5*step*(the_dot_i + the_dot_iMinus1));
//
//		// phi_eqn
//		fCon[2 * nh + i - 1] = phi_i - (phi_iMinus1 + 0.5*step*(phi_dot_i + phi_dot_iMinus1));
//
//		// u_rho_eqn
//		fCon[3 * nh + i - 1] = rho_dot_i - (rho_dot_iMinus1 + 0.5*step*(u_rho_i + u_rho_iMinus1) / l);
//
//		// u_the_eqn
//		fCon[4 * nh + i - 1] = the_dot_i - (the_dot_iMinus1 + 0.5*step*(u_the_i / I_the_i + u_the_iMinus1 / I_the_iMinus1));
//
//		// u_phi_eqn
//		fCon[5 * nh + i - 1] = phi_dot_i - (phi_dot_iMinus1 + 0.5*step*(u_phi_i / I_phi_i + u_phi_iMinus1 / I_phi_iMinus1));
//	}
//
//	return true;
//
//}
//
//bool LASO::ROBOT::conJac(double *x, double *jCon)
//{
//
//	integer nh = ncnln / 6;
//
//	doublereal step = (x[n - 1] / nh);
//
//	doublereal rho_i = 0.0;
//	doublereal rho_iMinus1 = 0.0;
//	doublereal the_i = 0.0;
//	doublereal the_iMinus1 = 0.0;
//	doublereal phi_i = 0.0;
//	doublereal phi_iMinus1 = 0.0;
//
//	doublereal rho_dot_i = 0.0;
//	doublereal rho_dot_iMinus1 = 0.0;
//	doublereal the_dot_i = 0.0;
//	doublereal the_dot_iMinus1 = 0.0;
//	doublereal phi_dot_i = 0.0;
//	doublereal phi_dot_iMinus1 = 0.0;
//
//	doublereal u_rho_i = 0.0;
//	doublereal u_rho_iMinus1 = 0.0;
//	doublereal u_the_i = 0.0;
//	doublereal u_the_iMinus1 = 0.0;
//	doublereal u_phi_i = 0.0;
//	doublereal u_phi_iMinus1 = 0.0;
//
//	doublereal I_the_i = 0.0;
//	doublereal I_the_iMinus1 = 0.0;
//
//	doublereal I_phi_i = 0.0;
//	doublereal I_phi_iMinus1 = 0.0;
//
//	integer iStart = 1;
//	integer iEnd = nh + 1;
//	for (integer i = iStart; i < iEnd; ++i)
//	{
//		rho_i = x[0 * (nh + 1) + i];
//		rho_iMinus1 = x[0 * (nh + 1) + i - 1];
//		the_i = x[1 * (nh + 1) + i];
//		the_iMinus1 = x[1 * (nh + 1) + i - 1];
//		phi_i = x[2 * (nh + 1) + i];
//		phi_iMinus1 = x[2 * (nh + 1) + i - 1];
//
//		rho_dot_i = x[3 * (nh + 1) + i];
//		rho_dot_iMinus1 = x[3 * (nh + 1) + i - 1];
//		the_dot_i = x[4 * (nh + 1) + i];
//		the_dot_iMinus1 = x[4 * (nh + 1) + i - 1];
//		phi_dot_i = x[5 * (nh + 1) + i];
//		phi_dot_iMinus1 = x[5 * (nh + 1) + i - 1];
//
//		u_rho_i = x[6 * (nh + 1) + i];
//		u_rho_iMinus1 = x[6 * (nh + 1) + i - 1];
//		u_the_i = x[7 * (nh + 1) + i];
//		u_the_iMinus1 = x[7 * (nh + 1) + i - 1];
//		u_phi_i = x[8 * (nh + 1) + i];
//		u_phi_iMinus1 = x[8 * (nh + 1) + i - 1];
//
//		I_the_i = (pow(l - rho_i, 3.0) + pow(rho_i, 3.0)) * pow(sin(phi_i), 2.0) / 3.0;
//		I_the_iMinus1 = (pow(l - rho_iMinus1, 3.0) + pow(rho_iMinus1, 3.0)) * pow(sin(phi_iMinus1), 2.0) / 3.0;
//
//		I_phi_i = (pow(l - rho_i, 3.0) + pow(rho_i, 3)) / 3.0;
//		I_phi_iMinus1 = (pow(l - rho_iMinus1, 3.0) + pow(rho_iMinus1, 3)) / 3.0;
//
//		integer jStart = 0;
//		integer jEnd = nh + 1;
//		for (integer j = jStart; j < jEnd; ++j)
//		{
//			// rho_eqn
//			jCon[j * ncnln + (0 * nh + i - 1)] = rho_i - (rho_iMinus1 + 0.5*step*(rho_dot_i + rho_dot_iMinus1));
//
//			// the_eqn
//			jCon[j * ncnln + (1 * nh + i - 1)] = the_i - (the_iMinus1 + 0.5*step*(the_dot_i + the_dot_iMinus1));
//
//			// phi_eqn
//			jCon[j * ncnln + (2 * nh + i - 1)] = phi_i - (phi_iMinus1 + 0.5*step*(phi_dot_i + phi_dot_iMinus1));
//
//			// u_rho_eqn
//			jCon[j * ncnln + (3 * nh + i - 1)] = rho_dot_i - (rho_dot_iMinus1 + 0.5*step*(u_rho_i + u_rho_iMinus1) / l);
//
//			// u_the_eqn
//			jCon[j * ncnln + (4 * nh + i - 1)] = the_dot_i - (the_dot_iMinus1 + 0.5*step*(u_the_i / I_the_i + u_the_iMinus1 / I_the_iMinus1));
//
//			// u_phi_eqn
//			jCon[j * ncnln + (5 * nh + i - 1)] = phi_dot_i - (phi_dot_iMinus1 + 0.5*step*(u_phi_i / I_phi_i + u_phi_iMinus1 / I_phi_iMinus1));
//		}
//	}
//
//	// Note: Jacobian is stored in single dimensional array.
//	// Numbering system is as follows:
//	// [design variable number * total number of constraints + constraint number]
//	jCon[0 * 2 + 0] = -3.0 * x[0] * x[0];	// Derivative of the first constraint wrt variable 1.
//	jCon[1 * 2 + 0] = 1.0;					// Derivative of the first constraint wrt variable 2.
//	jCon[2 * 2 + 0] = -2.0 * x[2];			// Derivative of the first constraint wrt variable 3.
//	jCon[3 * 2 + 0] = 0.0;					// Derivative of the first constraint wrt variable 4.
//
//	jCon[0 * 2 + 1] = 2.0 * x[0];			// Derivative of the second constraint wrt variable 1.
//	jCon[1 * 2 + 1] = -1.0;					// Derivative of the second constraint wrt variable 2.
//	jCon[2 * 2 + 1] = 0.0;					// Derivative of the second constraint wrt variable 3.
//	jCon[3 * 2 + 1] = -2.0 * x[3];			// Derivative of the second constraint wrt variable 4.
//
//	return true;
//
//}