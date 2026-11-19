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

#include "model.h"
#include "environment.h"
#include <math.h>
#include <uxhw.h>

float
speedDistribution(Model * model)
{
	return UxHwFloatGaussDist(model->speedExpected, model->speedStd);
}

float
moveModel(Model * model, float position)
{
	return position + speedDistribution(model);
}

float
sensorModel(Model * model, Environment *  env, float position)
{
	float   closest = INFINITY;
	float   distance;
	float   distanceWithNoise;

	for (size_t i = 0; i < getNumberOfLandmarks(env); i++)
	{
		distance            = fabs(getLandmarkPosition(env, i) - position);
		distanceWithNoise   = UxHwFloatGaussDist(distance, distance * model->sensorStdCoeff);
		closest             = fmin(closest, distanceWithNoise);
	}

	return closest;
}
