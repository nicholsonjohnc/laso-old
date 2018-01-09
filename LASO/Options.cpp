#include "Options.h"

// Define default constructor for Options class. 
LASO::Options::Options()
{

	majorOptimalityTolerance = 1.1e-8;	// The major iteration optimality tolerance.
	minorOptimalityTolerance = 1.1e-8;	// The minor iteration optimality tolerance.
	majorFeasibilityTolerance = 1.1e-8;	// The major iteration feasibility tolerance.
	minorFeasibilityTolerance = 1.1e-8;	// The minor iteration feasibility tolerance.
	rhoFactor = 1.0e-4;					// A factor between 0 and 1 used by Armijo's rule in determining satisfactory step sizes.
	output = false;						// Toggle output on / off.
	majorIterationsLimit = 500;			// Maximum number of iterations.
	minorIterationsLimit = 500;			// Maximum number of minor iterations.
	noCorrections = 20;					// Number of s and y corrections to store for l-bfgs design variable search direction calculations.
	penalty = 0.01;						// Initial penalty parameter used for all constraints.
	scaling = 1.0;						// Initial search direction scaling.
	infiniteBoundSize = 1.0e20;			// Size at which to consider bound non-existent.

}