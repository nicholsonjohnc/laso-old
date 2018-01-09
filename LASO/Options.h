#pragma once

#ifndef LASO_OPTIONS
#define LASO_OPTIONS

#include "Common.h"

namespace LASO
{
	// Declare Options class. 
	class Options
	{

	public:

		// Declare public attributes.
		doublereal majorOptimalityTolerance;	// The major iteration optimality tolerance.
		doublereal minorOptimalityTolerance;	// The minor iteration optimality tolerance.
		doublereal majorFeasibilityTolerance;	// The major iteration feasibility tolerance.
		doublereal minorFeasibilityTolerance;	// The minor iteration feasibility tolerance.
		doublereal rhoFactor;					// A factor between 0 and 1 used by Armijo's rule in determining satisfactory step sizes.
		boolean output;							// Toggle output on / off.
		integer majorIterationsLimit;			// Maximum number of iterations.
		integer minorIterationsLimit;			// Maximum number of minor iterations.
		integer noCorrections;					// Number of s and y corrections to store for l-bfgs design variable search direction calculations.
		doublereal penalty;						// Initial penalty parameter used for all constraints.
		doublereal scaling;						// Initial search direction scaling.
		doublereal infiniteBoundSize;			// Size at which to consider bound non-existent.
		Options();								// Default constructor.

	private:

		// Declare private attributes.

	};

}

#endif 