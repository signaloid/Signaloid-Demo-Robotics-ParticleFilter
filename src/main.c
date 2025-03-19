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
#include <stdlib.h>
#include <uxhw.h>

#include "utilities.h"
#include "environment.h"
#include "model.h"
#include "plot.h"
#include "system.h"

int
main(int argc, char * argv[])
{
	CommandLineArguments			arguments = {0};
	Environment				env = {0};
	Model					model = {0};
	System					system = {0};
	SensorModelEvidenceArgumentType		sensorModelArgs;
	FILE *					inputFile;
	FILE *					outputFile;
	bool					svgMode = false;

	/*
	 *	Estimation of the state (distribution).
	 */
	float			estimatedPosition;
	/*
	 *	Count the iteration for printing.
	 */
	size_t			steps = 0;

	/*
	 *	User supplied configuration.
	 */
	if (getCommandLineArguments(argc, argv, &arguments) != kCommonConstantReturnTypeSuccess)
	{
		return 1;
	}

	/*
	 *	Read landmark positions from the input file.
	 */
	if (arguments.common.isInputFromFileEnabled)
	{
		inputFile = fopen(arguments.common.inputFilePath, "r");
		if (inputFile == NULL)
		{
			fprintf(stderr, "Could not open %s\n", arguments.common.inputFilePath);

			return 1;
		}

		readLandmarkPositionsFromInputFile(inputFile, &env);

		fclose(inputFile);
	}

	/*
	 *	If the output file is specified, open it and redirect output to the output file.
	 *	Enables SVG-mode.
	 */
	if (arguments.common.isWriteToFileEnabled)
	{
		outputFile = fopen(arguments.common.outputFilePath, "w");
		if (outputFile == NULL)
		{
			fprintf(stderr, "Could not open %s\n", arguments.common.outputFilePath);

			return 1;
		}
		svgMode = true;
	}

	/*
	 *	Initializations
	 */
	model.speedStd = arguments.speedStd;
	model.speedExpected = arguments.speedExpected;
	model.sensorStdCoeff = arguments.sensorStdCoeff;

	env.positionMin = arguments.positionMin;
	env.positionMax = arguments.positionMax;

	estimatedPosition = UxHwFloatUniformDist(env.positionMin, env.positionMax);
	system.position = UxHwFloatSample(estimatedPosition);
	system.speed = UxHwFloatSample(UxHwFloatGaussDist(model.speedExpected, model.speedStd));

	/*
	 *	If no landmark was read, specify a landmark position in the middle of the allowed positions.
	 */
	if (env.landmarkNumber == 0)
	{
		addLandmark(&env, (env.positionMin + env.positionMax) / 2);
	}

	/*
	 *	Print what does not change.
	 */
	if (!svgMode)
	{
		printf("Speed: %f\n", system.speed);
		for (size_t i = 0; i < getNumberOfLandmarks(&env); i++)
		{
			printf("Landmark at %f\n", getLandmarkPosition(&env, i));
		}
	}
	else
	{
		plotHeader(outputFile, &env, &model, &system);
	}

	/*
	 *	Main computational loop
	 */
	while (	(system.position > env.positionMin) &&
		(system.position < env.positionMax) &&
		(steps < arguments.maxNumberOfSteps))
	{
		float sensorReading;

		/*
		 *	Print the system and filter state.
		 */
		if (!svgMode)
		{
			printf("\n# Step %zu\n", steps);
			printf("position's prior: %f\n", estimatedPosition);
		}
		else
		{
			plotFrameBeforeBL(outputFile, &env, &system, steps, estimatedPosition);
		}

		/*
		 *	Get a reading from the sensor.
		 */
		sensorReading = sense(&system, &model, &env);

		/*
		 *	Apply Bayes-Laplace rule with the new observation to better estimate the state.
		 */
		sensorModelArgs.m = &model;
		sensorModelArgs.env = &env;
		estimatedPosition = UxHwFloatBayesLaplace(
			&sensorModelEvidence,
			(void *)(&sensorModelArgs),
			estimatedPosition,
			sensorReading,
			1);

		/*
		 *	Print the system and filter state.
		 */
		if (!svgMode)
		{
			printf("position: %f\n", system.position);
			printf("estimated position: %f\n", estimatedPosition);
			printf("1st moment of the estimated position: %f\n",
			       UxHwFloatNthMoment(estimatedPosition, 1));
		}
		else
		{
			plotFrameAfterBL(outputFile, &env, &model, &system, steps, estimatedPosition, sensorReading);
		}

		/*
		 *	Update the system's position by simulating the motion.
		 */
		move(&system);

		/*
		 *	Update the estimated position using a model of the system's motion.
		 */
		estimatedPosition = moveModel(&model, estimatedPosition);

		steps++;
	}

	if (svgMode)
	{
		plotFooter(outputFile);
	}

	if (arguments.common.isWriteToFileEnabled)
	{
		fclose(outputFile);
	}

	return 0;
}
