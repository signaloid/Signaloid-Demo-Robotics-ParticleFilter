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

#include <stdio.h>
#include <uxhw.h>
#include <string.h>
#include "model.h"
#include "environment.h"
#include "utilities.h"

void
printUsage()
{
	fprintf(stderr, "Example: Particle Filter - Signaloid version\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Usage: Valid command-line arguments are:\n");
	fprintf(
		stderr,
		"\t[-i, --input <Path to the landmarks input file : str>] (Read landmarks from file.)\n"
		"\t[-o, --output <Path to the SVG output file : str>] (Specify the svg output file.)\n"
		"\t[-S, --select-output <output : int>] (Compute 0-indexed output, by default 0.)\n"
		"\t[-h, --help] (Display this help message.)\n");
	fprintf(
		stderr,
		"\t[-k, --max-number-of-steps <Maximum number of steps to execute : int> (Default: %d)]\n"
		"\t[-p, --position-min <Min position in X-axis : float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-P, --position-max <Max position in X-axis: float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-e, --expected-speed <The expected speed of the model: float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-s, --speed-standard-deviation <The standard deviaton of the speed of the model : float> (Default: %"SignaloidParticleModifier"f)]\n"
		"\t[-c, --sensor-coefficient <The standard deviaton coefficient of the speed of the model : float> (Default: %"SignaloidParticleModifier"f)]\n",
		kConfigurationConstantMaxFilterStepsDefault,
		kEnvironmentConstantDefaultPositionMin,
		kEnvironmentConstantDefaultPositionMax,
		kModelConstantDefaultSpeedExpected,
		kModelConstantDefaultSpeedStd,
		kModelConstantDefaultSensorStdCoeff);
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
setDefaultCommandLineArguments(CommandLineArguments *  arguments)
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

	arguments->maxNumberOfSteps	= kConfigurationConstantMaxFilterStepsDefault;
	arguments->positionMin		= kEnvironmentConstantDefaultPositionMin;
	arguments->positionMax		= kEnvironmentConstantDefaultPositionMax;
	arguments->speedExpected	= kModelConstantDefaultSpeedExpected;
	arguments->speedStd		= kModelConstantDefaultSpeedStd;
	arguments->sensorStdCoeff	= kModelConstantDefaultSensorStdCoeff;

#pragma GCC diagnostic pop

	return kCommonConstantReturnTypeSuccess;
}

CommonConstantReturnType
getCommandLineArguments(int argc, char *  argv[], CommandLineArguments *  arguments)
{
	const char *	maxNubmerOfStepsVariableArg = NULL;
	const char *	positionMinVariableArg = NULL;
	const char *	positionMaxVariableArg = NULL;
	const char *	speedExpectedVariableArg = NULL;
	const char *	speedStdVariableArg = NULL;
	const char *	sensorStdCoeffVariableArg = NULL;

	if (arguments == NULL)
	{
		fprintf(stderr, "Error: The provided pointer to arguments is NULL.\n");

		return kCommonConstantReturnTypeError;
	}

	if (setDefaultCommandLineArguments(arguments) != kCommonConstantReturnTypeSuccess)
	{
		return kCommonConstantReturnTypeError;
	}

	DemoOption	options[] =
			{
				{ .opt = "k", .optAlternative = "max-number-of-steps",		.hasArg = true,		.foundArg = &maxNubmerOfStepsVariableArg,	.foundOpt = NULL },
				{ .opt = "p", .optAlternative = "position-min",			.hasArg = true,		.foundArg = &positionMinVariableArg,		.foundOpt = NULL },
				{ .opt = "P", .optAlternative = "position-max",			.hasArg = true,		.foundArg = &positionMaxVariableArg,		.foundOpt = NULL },
				{ .opt = "e", .optAlternative = "expected-speed",		.hasArg = true,		.foundArg = &speedExpectedVariableArg,		.foundOpt = NULL },
				{ .opt = "s", .optAlternative = "speed-standard-deviation",	.hasArg = true,		.foundArg = &speedStdVariableArg,		.foundOpt = NULL },
				{ .opt = "c", .optAlternative = "sensor-coefficient",		.hasArg = true,		.foundArg = &sensorStdCoeffVariableArg,		.foundOpt = NULL },
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

		return kCommonConstantReturnTypeError;
	}

	/*
	 *	If no output is selected, set outputSelect to kOutputDistributionIndexMax.
	 *	This triggers the demo to compute all outputs.
	 */
	if (!arguments->common.isOutputSelected)
	{
		arguments->common.outputSelect = 0;
	}

	/*
	 *	Selected output can never be greater than kOutputDistributionIndexMax.
	 */
	if (arguments->common.outputSelect > 0)
	{
		fprintf(stderr, "Error: Wrong output selection.\n");

		return kCommonConstantReturnTypeError;
	}

	/*
	 *	This demo does not support benchmarking mode.
	 */
	if (arguments->common.isBenchmarkingMode)
	{
		fprintf(stderr, "Error: Benchmarking mode is not supported.\n");

		return kCommonConstantReturnTypeError;
	}

	/*
	 *	This demo does not support timing mode.
	 */
	if (arguments->common.isTimingEnabled)
	{
		fprintf(stderr, "Error: Timing mode is not supported.\n");

		return kCommonConstantReturnTypeError;
	}

	/*
	 *	This demo does not support JSON output format.
	 */
	if (arguments->common.isOutputJSONMode)
	{
		fprintf(stderr, "Error: JSON output mode is not supported.\n");

		return kCommonConstantReturnTypeError;
	}

	/*
	 *	This demo does not support Monte Carlo Mode.
	 */
	if (arguments->common.isMonteCarloMode)
	{
		fprintf(stderr, "Error: Monte carlo mode is not supported.\n");

		return kCommonConstantReturnTypeError;
	}

	if (arguments->common.isVerbose)
	{
		fprintf(stderr, "Warning: Verbose mode not supported. Continuing in non-verbose mode.\n");
	}

	/*
	 *	Sanity check for command-line argument values.
	 */
	if (maxNubmerOfStepsVariableArg != NULL)
	{
		int numberOfSteps;

		if (parseIntChecked(maxNubmerOfStepsVariableArg, &numberOfSteps) != kCommonConstantReturnTypeSuccess)
		{
			fprintf(stderr, "Error: The numberOfSteps (-k) must be an integer.\n");
			printUsage();

			return kCommonConstantReturnTypeError;
		}

		if ((numberOfSteps <= 0))
		{
			fprintf(stderr, "Error: numberOfSteps must be >= 1\n");
			printUsage();

			return kCommonConstantReturnTypeError;
		}

		arguments->maxNumberOfSteps = numberOfSteps;
	}

	if (positionMinVariableArg != NULL)
	{
		float positionMin;

		if (parseFloatChecked(positionMinVariableArg, &positionMin) != kCommonConstantReturnTypeSuccess)
		{
			fprintf(stderr, "Error: The min position (-p) must be a floating point value.\n");
			printUsage();

			return kCommonConstantReturnTypeError;
		}

		arguments->positionMin = positionMin;
	}

	if (positionMaxVariableArg != NULL)
	{
		float positionMax;

		if (parseFloatChecked(positionMaxVariableArg, &positionMax) != kCommonConstantReturnTypeSuccess)
		{
			fprintf(stderr, "Error: The max position (-P) must be a floating point value.\n");
			printUsage();

			return kCommonConstantReturnTypeError;
		}

		arguments->positionMax = positionMax;
	}

	if (speedExpectedVariableArg != NULL)
	{
		float speedExpected;

		if (parseFloatChecked(speedExpectedVariableArg, &speedExpected) != kCommonConstantReturnTypeSuccess)
		{
			fprintf(stderr, "Error: The expected speed (-e) must be a floating point value.\n");
			printUsage();

			return kCommonConstantReturnTypeError;
		}

		arguments->speedExpected = speedExpected;
	}

	if (speedStdVariableArg != NULL)
	{
		float speedStd;

		if (parseFloatChecked(speedStdVariableArg, &speedStd) != kCommonConstantReturnTypeSuccess)
		{
			fprintf(stderr, "Error: The speed standard deviation (-s) must be a floating point value.\n");
			printUsage();

			return kCommonConstantReturnTypeError;
		}

		arguments->speedStd = speedStd;
	}

	if (sensorStdCoeffVariableArg != NULL)
	{
		float sensorStdCoeff;

		if (parseFloatChecked(sensorStdCoeffVariableArg, &sensorStdCoeff) != kCommonConstantReturnTypeSuccess)
		{
			fprintf(stderr, "Error: The max position (-P) must be a floating point value.\n");
			printUsage();

			return kCommonConstantReturnTypeError;
		}

		arguments->sensorStdCoeff = sensorStdCoeff;
	}

	return kCommonConstantReturnTypeSuccess;
}

float
sensorModelEvidence(void *  args, float x)
{
	Model *		m;
	Environment *	env;

	m = ((SensorModelEvidenceArgumentType *)args)->m;
	env = ((SensorModelEvidenceArgumentType *)args)->env;

	return sensorModel(m, env, x);
}

void
readLandmarkPositionsFromInputFile(FILE *  fp, Environment *  env)
{
	int	ret;
	float	position;

	while ((ret = fscanf(fp, "%f", &position)) > 0)
	{
		addLandmark(env, position);
	}

	return;
}
