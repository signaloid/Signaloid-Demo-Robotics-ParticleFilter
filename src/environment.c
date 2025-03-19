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

#include "environment.h"
#include <stdio.h>
#include <math.h>
#include "assert.h"

size_t
getNumberOfLandmarks(Environment *  env)
{
	return env->landmarkNumber;
}

void
addLandmark(Environment *  env, float position)
{
	if (env->landmarkNumber >= kEnvironmentConstantMaxLandmarks - 1)
	{
		fprintf(stderr, " [Warning] addLandmark: landmarkNumber >= kEnvironmentConstantMaxLandmarks - 1. Landmark will not be added\n");

		return;
	}

	env->landmarkPositions[env->landmarkNumber] = position;
	env->landmarkNumber++;

	return;
}

float
getLandmarkPosition(Environment *  env, size_t index)
{
	if (index < env->landmarkNumber)
	{
		return env->landmarkPositions[index];
	}
	else
	{
		fprintf(stderr, " [Warning] getLandmarkPosition: Index provided >= landmark number. Returning NAN...\n");

		return NAN;
	}
}
