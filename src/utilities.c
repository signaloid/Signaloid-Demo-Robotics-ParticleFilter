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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <uxhw.h>

#include "common.h"
#include "environment.h"
#include "kernel.h"
#include "model.h"
#include "plot.h"
#include "system.h"
#include "utilities.h"

void
printUsage(void)
{
	fprintf(stderr, "Example: Particle Filter - Signaloid version\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Usage: Valid command-line arguments are:\n");
	fprintf(
		stderr,
		"\t[-i, --input <Path to the landmarks input file : str>] (Read landmarks from file.)\n"
		"\t[-o, --output <Path to output CSV file : str>] (Specify the output file.)\n"
		"\t[-S, --select-output <output : int (Default: %d)>] (Compute 0-indexed output. Calculate all possible outputs if equal to %d.)\n"
		"\t[-M, --multiple-executions <Number of executions : int (Default: 1)>] (Repeated execute kernel for benchmarking.)\n"
		"\t[-T, --time] (Timing mode: Times and prints the timing of the kernel execution.)\n"
		"\t[-j, --json] (Print output in JSON format.)\n"
		"\t[-h, --help] (Display this help message.)\n",
		kParticleFilterOutputVariableIndexEstimatedPosition,
		kParticleFilterOutputVariableIndexMax
	);
	/*
	 *	Print demo specific options usage
	 */
	fprintf(
		stderr,
		"\t[-k, --max-number-of-steps <Maximum number of steps to execute : int> (Default: %d)]\n"
		"\t[-p, --position-min <Min position in X-axis : float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-P, --position-max <Max position in X-axis: float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-e, --expected-speed <The expected speed of the model: float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-s, --speed-standard-deviation <The standard deviaton of the speed of the model : float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-c, --sensor-coefficient <The standard deviaton coefficient of the speed of the model : float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-x, --true-position <The position the robot actually starts from : float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-y, --true-speed <The speed the robot actually moves at : float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-z, --sample-initial-state] (Draw the robot's initial position and speed instead of using the values above.)\n"
		"\t[-n, --sampled-readings] (Draw each sensor reading from the sensor model instead of using the noiseless distance.)\n"
		"\t[-g, --svg-output <Path to the SVG output file : str>] (Render the run as an SVG.)\n",
		kConfigurationConstantMaxFilterStepsDefault,
		kEnvironmentConstantDefaultPositionMin,
		kEnvironmentConstantDefaultPositionMax,
		kModelConstantDefaultSpeedExpected,
		kModelConstantDefaultSpeedStd,
		kModelConstantDefaultSensorStdCoeff,
		kParticleFilterDefaultTruePosition,
		kParticleFilterDefaultTrueSpeed
	);
	fprintf(stderr, "\n");

	return;
}

/**
 *	@brief	Set the default values for the command line arguments
 *
 *	@param	arguments	: command line arguments pointer
 *	@return			: `kCommonConstantReturnTypeSuccess` if successful, else `kCommonConstantReturnTypeError`
 */
static CommonConstantReturnType
setDefaultCommandLineArguments(CommandLineArguments * arguments)
{
	if (arguments == NULL)
	{
		fprintf(stderr, "Error: Arguments pointer is NULL.\n");

		return kCommonConstantReturnTypeError;
	}

/*
 *	Older GCC versions have a bug which gives a spurious warning for the C universal zero
 *	initializer `{0}`. Any workaround makes the code less portable or prevents the common code
 *	from adding new fields to the `CommonCommandLineArguments` struct. Therefore, we surpress
 *	this warning.
 *
 *	See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=53119.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"

	memset(&arguments->common, 0, sizeof(CommonCommandLineArguments));

	arguments->kernelParameters = (KernelParameters) {
		.maxNumberOfSteps   = kConfigurationConstantMaxFilterStepsDefault,
		.positionMin        = kEnvironmentConstantDefaultPositionMin,
		.positionMax        = kEnvironmentConstantDefaultPositionMax,
		.speedExpected      = kModelConstantDefaultSpeedExpected,
		.speedStd           = kModelConstantDefaultSpeedStd,
		.sensorStdCoeff     = kModelConstantDefaultSensorStdCoeff,
		.truePosition       = kParticleFilterDefaultTruePosition,
		.trueSpeed          = kParticleFilterDefaultTrueSpeed,
		.landmarkCount      = 0,
		.useSampledReadings = false,
		.trace              = NULL,
	};
	arguments->sampleInitialState   = false;
	arguments->svgOutputFilePath    = NULL;

#pragma GCC diagnostic pop

	return kCommonConstantReturnTypeSuccess;
}

#ifdef NO_OS_AVAILABLE
CommonConstantReturnType
setNoOSCommandLineArguments(CommandLineArguments * arguments)
{
	if (arguments == NULL)
	{
		fputs("Error: The provided pointer to arguments is NULL.\n", stderr);

		return kCommonConstantReturnTypeError;
	}

	/*
	 *	Start from the defaults so that every demo-specific field is
	 *	initialized, then override the ones the no-OS build fixes. The
	 *	`common` sub-struct is zeroed by this call, so it is set explicitly
	 *	below.
	 */
	if (setDefaultCommandLineArguments(arguments) != kCommonConstantReturnTypeSuccess)
	{
		return kCommonConstantReturnTypeError;
	}

	arguments->common.numberOfMonteCarloIterations  = 1;
	arguments->common.outputSelect                  = kParticleFilterOutputVariableIndexMax;
	arguments->common.isTimingEnabled               = false;
	arguments->common.isMonteCarloMode              = false;
	arguments->common.isOutputJSONMode              = false;
	arguments->common.isWriteToFileEnabled          = false;

	/*
	 *	Every demo-specific parameter keeps its default, so a no-OS run
	 *	computes what an OS run with no arguments computes. Anything changed
	 *	here has to be changed in the ground-truth references that the
	 *	benchmarking repositories compare no-OS runs against.
	 */

	return kCommonConstantReturnTypeSuccess;
}
#endif

CommonConstantReturnType
getCommandLineArguments(
	int                     argc,
	char *                  argv[],
	CommandLineArguments *  arguments)
{
#ifdef NO_OS_AVAILABLE
	/*
	 *	The no-OS build has no command line to parse, so ignore `argc` and
	 *	`argv` and use the hard-coded configuration instead.
	 */
	(void) argc;
	(void) argv;

	puts("Using hard coded command line arguments");

	return setNoOSCommandLineArguments(arguments);

#else
	const char *    maxNumberOfStepsArg     = NULL;
	const char *    positionMinArg          = NULL;
	const char *    positionMaxArg          = NULL;
	const char *    speedExpectedArg        = NULL;
	const char *    speedStdArg             = NULL;
	const char *    sensorStdCoeffArg       = NULL;
	const char *    truePositionArg         = NULL;
	const char *    trueSpeedArg            = NULL;
	const char *    svgOutputArg            = NULL;
	bool            sampleInitialStateFound = false;
	bool            sampledReadingsFound    = false;

	if (arguments == NULL)
	{
		fprintf(stderr, "Error: The provided pointer to arguments is NULL.\n");

		return kCommonConstantReturnTypeError;
	}

	if (setDefaultCommandLineArguments(arguments) != kCommonConstantReturnTypeSuccess)
	{
		return kCommonConstantReturnTypeError;
	}

	DemoOption options[] = {
		{ .opt = "k", .optAlternative = "max-number-of-steps",      .hasArg = true,  .foundArg = &maxNumberOfStepsArg, .foundOpt = NULL                     },
		{ .opt = "p", .optAlternative = "position-min",             .hasArg = true,  .foundArg = &positionMinArg,      .foundOpt = NULL                     },
		{ .opt = "P", .optAlternative = "position-max",             .hasArg = true,  .foundArg = &positionMaxArg,      .foundOpt = NULL                     },
		{ .opt = "e", .optAlternative = "expected-speed",           .hasArg = true,  .foundArg = &speedExpectedArg,    .foundOpt = NULL                     },
		{ .opt = "s", .optAlternative = "speed-standard-deviation", .hasArg = true,  .foundArg = &speedStdArg,         .foundOpt = NULL                     },
		{ .opt = "c", .optAlternative = "sensor-coefficient",       .hasArg = true,  .foundArg = &sensorStdCoeffArg,   .foundOpt = NULL                     },
		{ .opt = "x", .optAlternative = "true-position",            .hasArg = true,  .foundArg = &truePositionArg,     .foundOpt = NULL                     },
		{ .opt = "y", .optAlternative = "true-speed",               .hasArg = true,  .foundArg = &trueSpeedArg,        .foundOpt = NULL                     },
		{ .opt = "z", .optAlternative = "sample-initial-state",     .hasArg = false, .foundArg = NULL,                 .foundOpt = &sampleInitialStateFound },
		{ .opt = "n", .optAlternative = "sampled-readings",         .hasArg = false, .foundArg = NULL,                 .foundOpt = &sampledReadingsFound    },
		{ .opt = "g", .optAlternative = "svg-output",               .hasArg = true,  .foundArg = &svgOutputArg,        .foundOpt = NULL                     },
		{ 0 },
	};

	if (parseArgs(argc, argv, &arguments->common, options) != kCommonConstantReturnTypeSuccess)
	{
		fprintf(stderr, "Error: Parsing command line arguments failed.\n");
		printUsage();

		return kCommonConstantReturnTypeError;
	}

	if (arguments->common.isHelpEnabled)
	{
		printUsage();

		exit(EXIT_SUCCESS);
	}

	/*
	 *	Write to output file is not supported in MonteCarlo Mode.
	 */
	if (arguments->common.isWriteToFileEnabled && arguments->common.isMonteCarloMode)
	{
		fprintf(stderr, "Error: Writing to output file is not supported in MonteCarlo Mode.\n");

		return kCommonConstantReturnTypeError;
	}

	if (arguments->common.isVerbose)
	{
		fprintf(stderr, "Warning: Verbose mode not supported. Continuing in non-verbose mode.\n");
	}

	/*
	 *	If no output selected from CLA, set the print all value as default.
	 */
	if (!arguments->common.isOutputSelected)
	{
		arguments->common.outputSelect = kParticleFilterOutputVariableIndexEstimatedPosition;
	}

	/*
	 *	The output select value can be at most `kParticleFilterOutputVariableIndexMax`, which selects all outputs.
	 */
	if (arguments->common.outputSelect > kParticleFilterOutputVariableIndexMax)
	{
		fprintf(
			stderr,
			"Error: Output select value (-S option) is greater than the possible number of outputs: Provided %zd. Max: %d\n",
			arguments->common.outputSelect,
			kParticleFilterOutputVariableIndexMax
		);

		return kCommonConstantReturnTypeError;
	}
	/*
	 *	When all outputs are selected, we cannot be in Monte Carlo mode.
	 */
	else if (arguments->common.outputSelect == kParticleFilterOutputVariableIndexMax)
	{
		if (arguments->common.isMonteCarloMode)
		{
			fprintf(stderr, "Error: Please select a single output when in Monte Carlo mode.\n");

			return kCommonConstantReturnTypeError;
		}
	}

	/*
	 *	Handle demo-specific options.
	 */
	if (maxNumberOfStepsArg != NULL)
	{
		int numberOfSteps;

		if ((parseIntChecked(maxNumberOfStepsArg, &numberOfSteps) != kCommonConstantReturnTypeSuccess) ||
		    (numberOfSteps < 0))
		{
			fprintf(stderr, "Error: The maximum number of steps (-k) must be a non-negative integer.\n");
			printUsage();

			return kCommonConstantReturnTypeError;
		}

		arguments->kernelParameters.maxNumberOfSteps = (size_t) numberOfSteps;
	}

	/*
	 *	The six float-valued parameters parse identically, so run them from
	 *	one table rather than repeating the same block per option.
	 */
	struct
	{
		const char *    argument;
		float *         destination;
		const char *    description;
	}
	floatOptions[] = {
		{ positionMinArg,    &arguments->kernelParameters.positionMin,    "The minimum position (-p)"         },
		{ positionMaxArg,    &arguments->kernelParameters.positionMax,    "The maximum position (-P)"         },
		{ speedExpectedArg,  &arguments->kernelParameters.speedExpected,  "The expected speed (-e)"           },
		{ speedStdArg,       &arguments->kernelParameters.speedStd,       "The speed standard deviation (-s)" },
		{ sensorStdCoeffArg, &arguments->kernelParameters.sensorStdCoeff, "The sensor coefficient (-c)"       },
		{ truePositionArg,   &arguments->kernelParameters.truePosition,   "The true initial position (-x)"    },
		{ trueSpeedArg,      &arguments->kernelParameters.trueSpeed,      "The true speed (-y)"               },
	};

	for (size_t i = 0; i < sizeof(floatOptions) / sizeof(floatOptions[0]); i++)
	{
		float value;

		if (floatOptions[i].argument == NULL)
		{
			continue;
		}

		if (parseFloatChecked(floatOptions[i].argument, &value) != kCommonConstantReturnTypeSuccess)
		{
			fprintf(stderr, "Error: %s must be a floating point value.\n", floatOptions[i].description);
			printUsage();

			return kCommonConstantReturnTypeError;
		}

		*floatOptions[i].destination = value;
	}

	if (arguments->kernelParameters.positionMin >= arguments->kernelParameters.positionMax)
	{
		fprintf(stderr, "Error: The minimum position (-p) must be less than the maximum position (-P).\n");
		printUsage();

		return kCommonConstantReturnTypeError;
	}

	arguments->sampleInitialState                   = sampleInitialStateFound;
	arguments->kernelParameters.useSampledReadings  = sampledReadingsFound;
	arguments->svgOutputFilePath                    = svgOutputArg;

	return kCommonConstantReturnTypeSuccess;

#endif
}

#ifndef NO_OS_AVAILABLE
void
readLandmarkPositionsFromInputFile(FILE * fp, KernelParameters *  parameters)
{
	float position;

	while (fscanf(fp, "%f", &position) > 0)
	{
		if (parameters->landmarkCount >= kEnvironmentConstantMaxLandmarks)
		{
			fprintf(
				stderr,
				" [Warning] readLandmarkPositionsFromInputFile: more than %d landmarks in the input file. The rest will not be added\n",
				kEnvironmentConstantMaxLandmarks
			);

			return;
		}

		parameters->landmarkPositions[parameters->landmarkCount] = position;
		parameters->landmarkCount++;
	}

	return;
}

CommonConstantReturnType
writeRunAsSVG(
	const char *                filePath,
	const KernelParameters *    parameters,
	const ParticleFilterTrace * trace)
{
	Environment env     = { 0 };
	Model       model   = { 0 };
	System      system  = { 0 };
	FILE *      outputFile;

	/*
	 *	Rebuild the environment, model and system the run used, so that
	 *	`plot.c` can be handed them frame by frame. The kernel does no I/O,
	 *	so this replays the recorded trace rather than drawing as it goes.
	 */
	model.speedExpected     = parameters->speedExpected;
	model.speedStd          = parameters->speedStd;
	model.sensorStdCoeff    = parameters->sensorStdCoeff;

	env.positionMin = parameters->positionMin;
	env.positionMax = parameters->positionMax;

	for (size_t i = 0; i < parameters->landmarkCount; i++)
	{
		addLandmark(&env, parameters->landmarkPositions[i]);
	}

	if (getNumberOfLandmarks(&env) == 0)
	{
		addLandmark(&env, (env.positionMin + env.positionMax) / 2);
	}

	system.position = parameters->truePosition;
	system.speed    = parameters->trueSpeed;

	outputFile = fopen(filePath, "w");

	if (outputFile == NULL)
	{
		fprintf(stderr, "Error: Could not open %s\n", filePath);

		return kCommonConstantReturnTypeError;
	}

	plotHeader(outputFile, &env, &model, &system);

	for (size_t step = 0; step < trace->count; step++)
	{
		system.position = trace->steps[step].truePosition;

		plotFrameBeforeBL(
			outputFile,
			&env,
			&system,
			step,
			trace->steps[step].priorEstimatedPosition
		);
		plotFrameAfterBL(
			outputFile,
			&env,
			&model,
			&system,
			step,
			trace->steps[step].posteriorEstimatedPosition,
			trace->steps[step].sensorReading
		);
	}

	plotFooter(outputFile);
	fclose(outputFile);

	return kCommonConstantReturnTypeSuccess;
}
#endif
