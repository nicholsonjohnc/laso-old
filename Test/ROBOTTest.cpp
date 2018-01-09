//#include "CppUnitTest.h"
//
//#include "SolverAL.h"
//#include "ROBOT.h"
//
//using namespace Microsoft::VisualStudio::CppUnitTestFramework;
//
//namespace LASO
//{
//	TEST_CLASS(ROBOTTest)
//	{
//
//	public:
//
//		TEST_METHOD(ROBOT)
//		{
//
//			integer nh = 200;
//			doublereal L = 5.0;
//			doublereal pi = 4.0 * atan(1.0);
//			doublereal max_u_rho = 1.0;
//			doublereal max_u_the = 1.0;
//			doublereal max_u_phi = 1.0;
//
//			integer n = 9 * (nh + 1) + 1;
//			integer m = 6 * nh;
//			vector x(n, 0.0);
//			vector lb(n + m, 0.0);
//			vector ub(n + m, 0.0);
//			// Initialize design variables, lower bounds, and upper bounds.
//			integer iStart = 0;
//			integer iEnd = nh + 1;
//			for (integer i = iStart; i < iEnd; ++i)
//			{
//				// rho
//				x[0 * (nh + 1) + i] = 4.5;
//				lb[0 * (nh + 1) + i] = 0.0;
//				ub[0 * (nh + 1) + i] = L;
//
//				// the
//				x[1 * (nh + 1) + i] = (2.0 * pi / 3.0) * pow((doublereal)i / nh, 2.0);
//				lb[1 * (nh + 1) + i] = -pi;
//				ub[1 * (nh + 1) + i] = pi;
//
//				// phi
//				x[2 * (nh + 1) + i] = pi / 4.0;
//				lb[2 * (nh + 1) + i] = 0;
//				ub[2 * (nh + 1) + i] = pi;
//
//				// rho_dot
//				x[3 * (nh + 1) + i] = 0.0;
//				lb[3 * (nh + 1) + i] = -INFINITY;
//				ub[3 * (nh + 1) + i] = INFINITY;
//
//				// the_dot
//				x[4 * (nh + 1) + i] = (4.0 * pi / 3.0) * ((doublereal)i / nh);
//				lb[4 * (nh + 1) + i] = -INFINITY;
//				ub[4 * (nh + 1) + i] = INFINITY;
//
//				// phi_dot
//				x[5 * (nh + 1) + i] = 0.0;
//				lb[5 * (nh + 1) + i] = -INFINITY;
//				ub[5 * (nh + 1) + i] = INFINITY;
//
//				// u_rho
//				x[6 * (nh + 1) + i] = 0.0;
//				lb[6 * (nh + 1) + i] = -max_u_rho;
//				ub[6 * (nh + 1) + i] = max_u_rho;
//
//				// u_the
//				x[7 * (nh + 1) + i] = 0.0;
//				lb[7 * (nh + 1) + i] = -max_u_the;
//				ub[7 * (nh + 1) + i] = max_u_the;
//
//				// u_phi
//				x[8 * (nh + 1) + i] = 0.0;
//				lb[8 * (nh + 1) + i] = -max_u_phi;
//				ub[8 * (nh + 1) + i] = max_u_phi;
//			}
//
//			// rho_0_eqn
//			lb[0 * (nh + 1) + 0] = 4.5;
//			ub[0 * (nh + 1) + 0] = 4.5;
//
//			// the_0_eqn
//			lb[1 * (nh + 1) + 0] = 0.0;
//			ub[1 * (nh + 1) + 0] = 0.0;
//
//			// phi_0_eqn
//			lb[2 * (nh + 1) + 0] = pi / 4.0;
//			ub[2 * (nh + 1) + 0] = pi / 4.0;
//
//			// rho_f_eqn
//			lb[0 * (nh + 1) + nh] = 4.5;
//			ub[0 * (nh + 1) + nh] = 4.5;
//
//			// the_f_eqn
//			lb[1 * (nh + 1) + nh] = 2.0 * pi / 3.0;
//			ub[1 * (nh + 1) + nh] = 2.0 * pi / 3.0;
//
//			// phi_f_eqn
//			lb[2 * (nh + 1) + nh] = pi / 4.0;
//			ub[2 * (nh + 1) + nh] = pi / 4.0;
//
//			// rho_dot_0_eqn
//			lb[3 * (nh + 1) + 0] = 0.0;
//			ub[3 * (nh + 1) + 0] = 0.0;
//
//			// the_dot_0_eqn
//			lb[4 * (nh + 1) + 0] = 0.0;
//			ub[4 * (nh + 1) + 0] = 0.0;
//
//			// phi_dot_0_eqn
//			lb[5 * (nh + 1) + 0] = 0.0;
//			ub[5 * (nh + 1) + 0] = 0.0;
//
//			// rho_dot_f_eqn
//			lb[3 * (nh + 1) + nh] = 0.0;
//			ub[3 * (nh + 1) + nh] = 0.0;
//
//			// the_dot_f_eqn
//			lb[4 * (nh + 1) + nh] = 0.0;
//			ub[4 * (nh + 1) + nh] = 0.0;
//
//			// phi_dot_f_eqn
//			lb[5 * (nh + 1) + nh] = 0.0;
//			ub[5 * (nh + 1) + nh] = 0.0;
//
//			// tf
//			x[n - 1] = 1.0;
//			lb[n - 1] = -INFINITY;
//			ub[n - 1] = INFINITY;
//			
//			LASO::Options options;
//			vector fObj(1, 0.0);
//			vector gObj(n, 0.0);
//			vector fCon(m);
//			vector gCon(m * n);
//			options.output = true;
//			LASO::ROBOT robot(n, m, L);
//			LASO::ProblemAL problem(n, robot, &x[0], lb, ub, m, options, &fObj[0], &gObj[0], &fCon[0], &gCon[0]);
//			LASO::SolverAL solver;
//			solver.solve(problem);
//
//		}
//
//	};
//}