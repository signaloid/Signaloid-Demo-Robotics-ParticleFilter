/*
 *	Authored 2022, Damien Zufferey.
 *
 *	Copyright (c) 2022, Signaloid.
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

#include "config.h"
#include "environment.h"
#include "model.h"
#include "plot.h"
#include "system.h"

int
main(int argc, char * argv[])
{
	size_t i;
	/*
	 *	Estimation of the state (distribution)
	 */
	float estimatedPosition;
	/*
	 *	Count the iteration for printing
	 */
	size_t steps = 0;

	/*
	 *	User supplied configuration
	 */
	parseArguments(argc, argv);

	/*
	 *	Initialization
	 */
	estimatedPosition = UxHwFloatUniformDist(positionMin, positionMax);
	position = UxHwFloatSample(estimatedPosition);
	speed = UxHwFloatSample(UxHwFloatGaussDist(speedExpected, speedStd));

	/*
	 *	Print what does not change
	 */
	if (!plot)
	{
		printf("Speed: %f\n", speed);
		for (i = 0; i < getNumberOfLandmarks(); i++)
		{
			printf("Landmark at %f\n", getLandmarkPosition(i));
		}
	}
	else
	{
		plotHeader(stdout);
	}

	while (position > positionMin && position < positionMax && steps < maxFilterSteps)
	{
		/*
		 *	Print the system and filter state
		 */
		if (!plot)
		{
			printf("\n# Step %u\n", steps);
			printf("position's prior: %f\n", estimatedPosition);
		}
		else
		{
			plotFrameBeforeBL(stdout, steps, estimatedPosition);
		}

		/*
		 *	Get a reading from the sensor
		 */
		float sensorReading = sense();
		/*
		 *	Apply Bayes-Laplace rule with the new observation to better estimate the
		 *	state.
		 */
		estimatedPosition = UxHwFloatBayesLaplace(
			&sensorModel,
			estimatedPosition,
			sensorReading);

		/*
		 *	Print the system and filter state
		 */
		if (!plot)
		{
			printf("position: %f\n", position);
			printf("estimated position: %f\n", estimatedPosition);
			printf("1st mode of the estimated position: %f\n",
			       UxHwFloatNthMode(estimatedPosition, 1));
		}
		else
		{
			plotFrameAfterBL(stdout, steps, estimatedPosition, sensorReading);
		}

		/*
		 *	Update the system's position by simulating the motion
		 */
		move();

		/*
		 *	Update the estimated position using a model of the system's motion
		 */
		estimatedPosition = moveModel(estimatedPosition);

		steps++;
	}

	if (plot)
	{
		plotFooter(stdout);
	}

	return EXIT_SUCCESS;
}
