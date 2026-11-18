/*
 *	Copyright (c) 2022-2026, Signaloid.
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

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <uxhw.h>

#include "kernel.h"
#include "utilities.h"

#ifdef NO_OS_AVAILABLE

void
returnZeroNoOS(void);
#endif

int
main(int argc, char *  argv[])
{
	CommandLineArguments        arguments   = { 0 };
	ParticleFilterTrace         trace       = { 0 };

	double *                    monteCarloOutputSamples = NULL;
	clock_t                     start = 0;
	clock_t                     end;
	clock_t                     cpuTimeInClockTicks = 0;
	double                      outputVariables[kParticleFilterOutputVariableIndexMax] = { 0 };
	const char *                outputVariableNames[kParticleFilterOutputVariableIndexMax] = {
		"Estimated Position"
	};
	const char *                outputVariableDescriptions[kParticleFilterOutputVariableIndexMax] = {
		"Estimated position of the robot along the rail, after the last filter step."
	};
	kOutputVariableTypeIndex    outputVariableTypes[kParticleFilterOutputVariableIndexMax] = {
		kOutputVariableTypeDistribution
	};
	const char *                applicationDescription = "Robotics: 1D Particle Filter";
	MeanAndVariance             meanAndVariance;

	/*
	 *	Get command line arguments.
	 */
	if (getCommandLineArguments(argc, argv, &arguments) != kCommonConstantReturnTypeSuccess)
	{
		return kCommonConstantReturnTypeError;
	}

#ifndef NO_OS_AVAILABLE

	/*
	 *	Read landmark positions from the input file into the kernel
	 *	parameters, if one was given. This happens before timing starts, so
	 *	the timed region excludes file I/O. With no input file the kernel
	 *	places a single landmark in the middle of the allowed positions.
	 */
	if (arguments.common.isInputFromFileEnabled)
	{
		FILE * inputFile = fopen(arguments.common.inputFilePath, "r");

		if (inputFile == NULL)
		{
			fprintf(stderr, "Error: Could not open %s\n", arguments.common.inputFilePath);

			return kCommonConstantReturnTypeError;
		}

		readLandmarkPositionsFromInputFile(inputFile, &arguments.kernelParameters);
		fclose(inputFile);
	}

	/*
	 *	The SVG rendering replays what the kernel recorded, so the trace is
	 *	only allocated when an SVG was asked for.
	 */
	if (arguments.svgOutputFilePath != NULL)
	{
		trace.capacity  = arguments.kernelParameters.maxNumberOfSteps;
		trace.count     = 0;
		trace.steps     = (ParticleFilterTraceStep *) checkedMalloc(
			(trace.capacity > 0 ? trace.capacity : 1) * sizeof(ParticleFilterTraceStep),
			__FILE__,
			__LINE__
		);
		arguments.kernelParameters.trace = &trace;
	}
#endif

	/*
	 *	The drawn start, for anyone who wants the demo's original behaviour.
	 *	These are the only two `UxHwFloatSample()` calls outside the kernel,
	 *	and they are what makes such a run irreproducible.
	 */
	if (arguments.sampleInitialState)
	{
		arguments.kernelParameters.truePosition = UxHwFloatSample(
			UxHwFloatUniformDist(arguments.kernelParameters.positionMin, arguments.kernelParameters.positionMax)
		);
		arguments.kernelParameters.trueSpeed = UxHwFloatSample(
			UxHwFloatGaussDist(arguments.kernelParameters.speedExpected, arguments.kernelParameters.speedStd)
		);
	}

	/*
	 *	MonteCarlo output samples are used even in the UxHw use case to store
	 *	the result of the single distributional evaluation.
	 */
	monteCarloOutputSamples = (double *) checkedMalloc(
		(arguments.common.numberOfMonteCarloIterations > 0 ? arguments.common.numberOfMonteCarloIterations : 1) * sizeof(double),
		__FILE__,
		__LINE__
	);

	/*
	 *	Start timing.
	 */
	if (arguments.common.isTimingEnabled)
	{
		start = clock();
	}

	/*
	 *	Dispatch to the mode-specific kernel. The Monte Carlo loop lives
	 *	inside `calculateOutputMonteCarlo`; UxHw mode runs a
	 *	single distributional evaluation inside
	 *	`calculateOutputUxHw`.
	 */
	if (arguments.common.isMonteCarloMode)
	{
		calculateOutputMonteCarlo(
			&arguments.kernelParameters,
			arguments.common.numberOfMonteCarloIterations,
			outputVariables,
			monteCarloOutputSamples
		);

		/*
		 *	If not doing UxHw version, then approximate the cost of the third phase of
		 *	Monte Carlo (post-processing), by calculating the mean and variance.
		 */
		meanAndVariance = calculateMeanAndVarianceOfDoubleSamples(monteCarloOutputSamples, arguments.common.numberOfMonteCarloIterations);
		outputVariables[arguments.common.outputSelect] = meanAndVariance.mean;
	}
	else
	{
		calculateOutputUxHw(
			&arguments.kernelParameters,
			outputVariables,
			monteCarloOutputSamples
		);
	}

	/*
	 *	Stop timing.
	 */
	if (arguments.common.isTimingEnabled)
	{
		end                 = clock();
		cpuTimeInClockTicks = end - start;
	}

	/*
	 *	Print the results (either in JSON or standard output format).
	 */
	if (arguments.common.isOutputJSONMode)
	{
		printJSONFormattedOutput(
			&arguments.common,
			monteCarloOutputSamples,
			outputVariables,
			outputVariableDescriptions,
			kParticleFilterOutputVariableIndexMax,
			applicationDescription,
			outputVariableTypes
		);
	}
	else
	{
		printHumanConsumableOutput(
			&arguments.common,
			kParticleFilterOutputVariableIndexMax,
			outputVariables,
			outputVariableNames,
			outputVariableDescriptions,
			monteCarloOutputSamples,
			outputVariableTypes
		);
	}

	/*
	 *	Print timing result.
	 */
	if (arguments.common.isTimingEnabled)
	{
		printf("\nCPU time used: %"SignaloidParticleModifier"lf seconds\n", ((double) cpuTimeInClockTicks) / CLOCKS_PER_SEC);
	}

#ifndef NO_OS_AVAILABLE

	/*
	 *	Write output data.
	 */
	if (arguments.common.isWriteToFileEnabled)
	{
		if (writeOutputDoubleDistributionsToCSV(
				arguments.common.outputFilePath,
				outputVariables,
				outputVariableNames,
				kParticleFilterOutputVariableIndexMax
		))
		{
			free(monteCarloOutputSamples);
			free(trace.steps);

			return kCommonConstantReturnTypeError;
		}
	}

	/*
	 *	Render the run as an SVG, if one was asked for.
	 */
	if (arguments.svgOutputFilePath != NULL)
	{
		if (writeRunAsSVG(arguments.svgOutputFilePath, &arguments.kernelParameters, &trace) != kCommonConstantReturnTypeSuccess)
		{
			free(monteCarloOutputSamples);
			free(trace.steps);

			return kCommonConstantReturnTypeError;
		}
	}
#endif

	/*
	 *	Save Monte carlo outputs in an output file.
	 */
	if (arguments.common.isMonteCarloMode)
	{
		saveMonteCarloDoubleDataToDataDotOutFile(
			monteCarloOutputSamples, ((uint64_t) cpuTimeInClockTicks) * 1000000 / CLOCKS_PER_SEC,
			arguments.common.numberOfMonteCarloIterations
		);
	}

	/*
	 *	Free dynamically-allocated memory. `monteCarloOutputSamples` is
	 *	always allocated (even in UxHw mode, to hold the single
	 *	distributional result), so it is always freed here. `trace.steps` is
	 *	NULL unless an SVG was asked for, and `free(NULL)` is a no-op.
	 */
	free(monteCarloOutputSamples);
	free(trace.steps);

#ifdef NO_OS_AVAILABLE
	returnZeroNoOS();
#else

	return 0;

#endif
}
