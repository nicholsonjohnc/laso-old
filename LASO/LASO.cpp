#include "LASO.h"
#include "SolverAL.h"
#include "FunctionsSNOPT.h"

void LASO::Engine::solve(long int *n, double *x, funobjType funobj,
	double *lb, double *ub, long int *ncnln, funconType funcon, 
	double* fObj, double* gObj, double* fCon, double* gCon)
{
	// Initialize number of design variables.
	integer nDesignVariables = *n;

	// Initialize number of constraints
	integer mConstraints = *ncnln;

	// Initialize problem functions.
	LASO::FunctionsSNOPT functionsSNOPT(funobj, funcon, nDesignVariables, mConstraints, fObj, gObj, fCon, gCon);

	// Initialize design variables.
	vector xDesignVariables;
	xDesignVariables.resize(nDesignVariables);
	for (integer i = 0; i < nDesignVariables; ++i)
	{
		xDesignVariables[i] = x[i];
	}

	// Initialize lower and upper bounds.
	vector lbLowerBounds;
	vector ubUpperBounds;
	lbLowerBounds.resize(nDesignVariables + mConstraints);
	ubUpperBounds.resize(nDesignVariables + mConstraints);
	for (integer i = 0; i < nDesignVariables + mConstraints; ++i)
	{
		lbLowerBounds[i] = lb[i];
		ubUpperBounds[i] = ub[i];
	}

	// Initialize options.
	LASO::Options options;
	options.majorIterationsLimit = 100000;
	options.output = true;

	// Create problem.
	LASO::ProblemAL problem(nDesignVariables, functionsSNOPT, &xDesignVariables[0], lbLowerBounds, ubUpperBounds, mConstraints, options, fObj, gObj, fCon, gCon);

	// Create solver and solve problem.
	LASO::SolverAL solver;
	solver.solve(problem);

	// De-initialize design variables.
	for (integer i = 0; i < nDesignVariables; ++i)
	{
		x[i] = xDesignVariables[i];
	}
}