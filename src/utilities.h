/*
 *	Copyright (c) 2024-2026, Signaloid.
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

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "common.h"
#include "kernel.h"

typedef enum
{
	kConfigurationConstantMaxFilterStepsDefault = 5,
} ConfigurationConstant;

typedef struct
{
	CommonCommandLineArguments  common;
	KernelParameters            kernelParameters;
	/*
	 *	The true state of the simulated robot is scalar by default so that
	 *	a run is reproducible; `sampleInitialState` restores the drawn start
	 *	the demo used to have. `main()` draws it into `kernelParameters`
	 *	before timing starts.
	 */
	bool sampleInitialState;
	/*
	 *	Path of the SVG rendering of the run, or NULL for no SVG. The
	 *	common `-o` flag writes the output distributions as CSV, as it does
	 *	in every other demo, so the SVG has a flag of its own.
	 */
	const char * svgOutputFilePath;
} CommandLineArguments;

/**
 *	@brief	Print out command line usage.
 */
void
printUsage(void);

/**
 *	@brief	Get command line arguments.
 *
 *		In the no-OS build there is no command line: `argc` and `argv` are
 *		ignored and the hard-coded configuration set by
 *		`setNoOSCommandLineArguments()` is used instead.
 *
 *	@param	argc		: argument count from main().
 *	@param	argv		: argument vector from main().
 *	@param	arguments	: Pointer to struct to store arguments.
 *	@return			: `kCommonConstantReturnTypeSuccess` if successful,
 *				   else `kCommonConstantReturnTypeError`.
 */
CommonConstantReturnType
getCommandLineArguments(int argc, char *  argv[], CommandLineArguments *  arguments);

#ifdef NO_OS_AVAILABLE

/**
 *	@brief	Set the hard-coded command-line arguments used by the no-OS build.
 *
 *		This is the single place to change the configuration that no-OS
 *		runs use, since those runs cannot be given command-line arguments.
 *		Fields not set here keep the values from
 *		`setDefaultCommandLineArguments()`.
 *
 *	@param	arguments	: command-line arguments pointer.
 *	@return			: `kCommonConstantReturnTypeSuccess` if successful,
 *				   else `kCommonConstantReturnTypeError`.
 */
CommonConstantReturnType
setNoOSCommandLineArguments(CommandLineArguments * arguments);
#endif

#ifndef NO_OS_AVAILABLE

/**
 *	@brief	Reads landmark positions from the input file into the kernel
 *		parameters. The input file must be formatted as floating point
 *		values separated by white-space characters, with each value
 *		corresponding to a landmark position.
 *
 *	@param	fp		: The file pointer of the file to read from.
 *	@param	parameters	: The kernel parameters to add the landmarks to.
 */
void
readLandmarkPositionsFromInputFile(FILE * fp, KernelParameters *  parameters);

/**
 *	@brief	Render a filter run as an SVG, from the trace the kernel recorded.
 *
 *	@param	filePath	: Path of the SVG file to write.
 *	@param	parameters	: The kernel parameters the run used.
 *	@param	trace		: The trace the run recorded.
 *	@return			: `kCommonConstantReturnTypeSuccess` if successful,
 *				   else `kCommonConstantReturnTypeError`.
 */
CommonConstantReturnType
writeRunAsSVG(
	const char *                filePath,
	const KernelParameters *    parameters,
	const ParticleFilterTrace * trace);
#endif
