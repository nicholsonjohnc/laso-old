#pragma once

#ifndef LASO_COMMON
#define LASO_COMMON

#include <vector>
//#include <chrono>

typedef long int integer;								// Define type for an integer.
typedef double doublereal;								// Define type for a double.
typedef doublereal* pointer;							// Define type for a doublereal pointer.
typedef bool boolean;									// Define type for a boolean.
typedef char character;									// Define type for a character.
typedef std::string string;								// Define type for a string.
typedef std::vector<doublereal> vector;					// Define type for a 1D doublereal vector.
typedef std::vector <doublereal*> vectorpointer;		// Define type for a 1D doublereal pointer vector.
typedef std::vector<integer> vectorinteger;				// Define type for a 1D integer vector.
typedef std::vector<boolean> vectorboolean;				// Define type for a 1D boolean vector.
typedef std::vector<character> vectorcharacter;			// Define type for a 1D character vector.
typedef std::vector<std::vector<doublereal>> matrix;	// Define type for a 2D matrix.
//typedef std::chrono::high_resolution_clock highresolutionclock;		// Define type for a high-resolution clock.
//typedef std::chrono::steady_clock::time_point highresolutiontime;		// Define type for a high-resolution time.

#endif 
