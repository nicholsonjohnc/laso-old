#include <vector>
#include <iostream>

#include "LASO.h"

int funobj(long int *mode, long int *n, double *x, double *fObj, double *gObj, long int *nState);
int funcon(long int *mode, long int *ncnln, long int *n, long int *ldg, long int *needc, double *x, double *fCon, double *gCon, long int *nState);

int main() {

	long int n = 5;
	long int ncnln = 3;

	// initial values of the design variables
	double* x = new double[n];
	x[0] = 35.0;
	x[1] = -31.0;
	x[2] = 11.0;
	x[3] = 5.0;
	x[4] = -5.0;

	// lower bound on the design variables
	double* lb = new double[n + ncnln];
	lb[0] = -INFINITY;
	lb[1] = -INFINITY;
	lb[2] = -INFINITY;
	lb[3] = -INFINITY;
	lb[4] = -INFINITY;
	// lower bound on the constraints
	lb[5] = 0.0;
	lb[6] = 0.0;
	lb[7] = 0.0;

	// upper bound on the design variables
	double* ub = new double[n + ncnln];
	ub[0] = INFINITY;
	ub[1] = INFINITY;
	ub[2] = INFINITY;
	ub[3] = INFINITY;
	ub[4] = INFINITY;
	// upper bound on the constraints
	ub[5] = 0.0;
	ub[6] = 0.0;
	ub[7] = 0.0;

	double* fObj = new double[1];
	double* gObj = new double[n];
	double* fCon = new double[ncnln];
	double* gCon = new double[n * ncnln];

	LASO::Engine::solve(&n, x, funobj, lb, ub, &ncnln, funcon, fObj, gObj, fCon, gCon);

	delete[] x;
	delete[] lb;
	delete[] ub;
	delete[] fObj;
	delete[] gObj;
	delete[] fCon;
	delete[] gCon;

	return 0;

}

/**
* User provided subroutine funobj calculates:
*   - objective function, fObj, and
*   - objective function gradient, gObj
* at the current design variables, x
*/
int funobj(long int *mode, long int *n, double *x, double *fObj, double *gObj, long int *nState) {
	switch (*mode) {
	case 0:
		*fObj = pow(x[0] - x[1], 2.0) + pow(x[1] - x[2], 2.0) + pow(x[2] - x[3], 4.0) + pow(x[3] - x[4], 2.0);
		break;
	case 1:
		gObj[0] = 2.0*(x[0] - x[1]);
		gObj[1] = -2.0*(x[0] - x[1]) + 2.0*(x[1] - x[2]);
		gObj[2] = -2.0*(x[1] - x[2]) + 4.0*pow(x[2] - x[3], 3.0);
		gObj[3] = -4.0*pow(x[2] - x[3], 3.0) + 2.0*(x[3] - x[4]);
		gObj[4] = -2.0*(x[3] - x[4]);
		break;
	}
	return 0;
}

/**
* User provided subroutine funcon calculates
*   - constraint functions, fCon, and
*   - constraint Jacobian, gCon,
* at the current design variables, x
*/
int funcon(long int *mode, long int *ncnln, long int *n, long int *ldg,
	long int *needc, double *x, double *fCon, double *gCon, long int *nState) {
	switch (*mode) {
	case 0:
		fCon[0] = x[0] + 2.0*x[1] + 3.0*x[2] - 6.0;
		fCon[1] = x[1] + 2.0*x[2] + 3.0*x[3] - 6.0;
		fCon[2] = x[2] + 2.0*x[3] + 3.0*x[4] - 6.0;
		break;
	case 1:
		// Note: Jacobian is stored in single dimensional array.
		// Numbering system is as follows:
		// [design variable number * total number of constraints + constraint number]
		gCon[0 * (*ncnln) + 0] = 1.0; // derivative of the first constraint wrt variable 1
		gCon[1 * (*ncnln) + 0] = 2.0; // derivative of the first constraint wrt variable 2
		gCon[2 * (*ncnln) + 0] = 3.0; // derivative of the first constraint wrt variable 3
		gCon[3 * (*ncnln) + 0] = 0.0; // derivative of the first constraint wrt variable 4
		gCon[4 * (*ncnln) + 0] = 0.0; // derivative of the first constraint wrt variable 5

		gCon[0 * (*ncnln) + 1] = 0.0; // derivative of the second constraint wrt variable 1
		gCon[1 * (*ncnln) + 1] = 1.0; // derivative of the second constraint wrt variable 2
		gCon[2 * (*ncnln) + 1] = 2.0; // derivative of the second constraint wrt variable 3
		gCon[3 * (*ncnln) + 1] = 3.0; // derivative of the second constraint wrt variable 4
		gCon[4 * (*ncnln) + 1] = 0.0; // derivative of the second constraint wrt variable 5

		gCon[0 * (*ncnln) + 2] = 0.0; // derivative of the third constraint wrt variable 1
		gCon[1 * (*ncnln) + 2] = 0.0; // derivative of the third constraint wrt variable 2
		gCon[2 * (*ncnln) + 2] = 1.0; // derivative of the third constraint wrt variable 3
		gCon[3 * (*ncnln) + 2] = 2.0; // derivative of the third constraint wrt variable 4
		gCon[4 * (*ncnln) + 2] = 3.0; // derivative of the third constraint wrt variable 5
		break;
	}
	return 0;
}





//#include <Eigen/Dense>
//
//int main() {
//
//
//	//int n = 3;
//	//Eigen::VectorXd x(n), b(n);
//	//Eigen::SparseMatrix<double> A(n, n);
//
//	//// fill A and b
//	//A.coeffRef(0, 0) = 1.0;
//	//A.coeffRef(0, 1) = 2.0;
//	//A.coeffRef(0, 2) = 3.0;
//	//A.coeffRef(1, 0) = 4.0;
//	//A.coeffRef(1, 1) = 5.0;
//	//A.coeffRef(1, 2) = 6.0;
//	//A.coeffRef(2, 0) = 7.0;
//	//A.coeffRef(2, 1) = 8.0;
//	//A.coeffRef(2, 2) = 10.0;
//
//	//b.coeffRef(0) = 3;
//	//b.coeffRef(1) = 3;
//	//b.coeffRef(1) = 4;
//
//	//Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower | Eigen::Upper> cg;
//	//cg.setMaxIterations(100);
//
//	//cg.compute(A);
//	//x = cg.solve(b);
//	//std::cout << "#iterations:     " << cg.iterations() << std::endl;
//	//std::cout << "estimated error: " << cg.error() << std::endl;
//	//// update b, and solve again
//	////x = cg.solve(b);
//
//	//std::cout << A << std::endl;
//	//std::cout << b << std::endl;
//	//std::cout << x << std::endl;
//
//
//
//	//Eigen::MatrixXd m(2, 2);
//	//m(0, 0) = 3;
//	//m(1, 0) = 2.5;
//	//m(0, 1) = -1;
//	//m(1, 1) = m(1, 0) + m(0, 1);
//	//std::cout << m << std::endl;
//
//
//	Eigen::MatrixXd A(3, 3);
//	Eigen::VectorXd b(3);
//
//	// fill A and b
//	A(0, 0) = 1.0;
//	A(0, 1) = 2.0;
//	A(0, 2) = 3.0;
//	A(1, 0) = 4.0;
//	A(1, 1) = 5.0;
//	A(1, 2) = 6.0;
//	A(2, 0) = 7.0;
//	A(2, 1) = 8.0;
//	A(2, 2) = 10.0;
//
//	b(0) = 3;
//	b(1) = 3;
//	b(2) = 4;
//
//	Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
//
//	std::cout << "Here is the matrix A:\n" << A << std::endl;
//	std::cout << "Here is the right hand side b:\n" << b << std::endl;
//	std::cout << "The least-squares solution is:\n" << x << std::endl;
//
//}