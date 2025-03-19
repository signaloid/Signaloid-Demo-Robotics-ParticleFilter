/*
 *	Copyright (c) 2024, Signaloid.
 *
 *	Permission is hereby granted, free of charge, to any person obtaining a copy
 *	of this software and associated documentation files (the "Software"), to deal
 *	in the Software without restriction, including without limitation the rights
 *	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *	copies of the Software, and to permit persons to whom the Software is
 *	furnished to do so, subject to the following conditions:
 *
 *	The above copyright notice and this permission notice shall be included in all
 *	copies or substantial portions of the Software.
 *
 *	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *	SOFTWARE.
 */

#include <stddef.h>
#include <stdio.h>
#include "common.h"
#include "model.h"
#include "environment.h"

typedef enum
{
	kConfigurationConstantMaxFilterStepsDefault = 5,
} ConfigurationConstant;

typedef struct
{
	Model *		m;
	Environment *	env;
} SensorModelEvidenceArgumentType;

typedef struct
{
	CommonCommandLineArguments	common;
	int				maxNumberOfSteps;
	float				positionMin;
	float				positionMax;
	float				speedExpected;
	float				speedStd;
	float				sensorStdCoeff;
} CommandLineArguments;


/**
 *	@brief	Print out command line usage.
 */
void	printUsage(void);

/**
 *	@brief	Get command line arguments.
 *
 *	@param	argc		: argument count from main()
 *	@param	argv		: argument vector from main()
 *	@param	arguments	: Pointer to struct to store arguments
 *	@return			: `kCommonConstantReturnTypeSuccess` if successful, else `kCommonConstantReturnTypeError`
 */
CommonConstantReturnType
getCommandLineArguments(int argc, char *  argv[], CommandLineArguments *  arguments);

/**
 *	@brief	Auxiliary function acting as a wrapper for calling `sensorModel` function
 *		as an evidence function through the `UxHwBayesLaplace`.
 *
 *	@param	args		: A pointer pointing to the necessary arguments of the `sensorModel` typecasted to a void*.
 *	@param	x		: The float argument of  the evidence function.
 *	@return	float		: Returns the value returned by `sensorModel` function.
 */
float	sensorModelEvidence(void *  args, float x);

/**
 *	@brief	Reads landmark positions from the input file. Input file must be formatted as
 *		floating point values separated by white-space characters, with each value
 *		corresponding to a landmark position.
 *
 *	@param	fp		: The file pointer of the file to read from.
 *	@param	env		: The pointer to the environment to add the landmarks to.
 */
void	readLandmarkPositionsFromInputFile(FILE *  fp, Environment *  env);
