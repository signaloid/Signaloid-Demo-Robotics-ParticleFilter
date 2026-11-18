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

#include "environment.h"
#include <math.h>

/*
 *	This file is part of the kernel's dependencies, so it is compiled into
 *	bare-metal builds -- the Signaloid SoC application that runs the filter on a
 *	C0 compute module, among them -- which have no stderr to write to. The two
 *	out-of-range cases below are therefore handled silently, by refusing the
 *	landmark and by returning NaN respectively, rather than diagnosed.
 */
size_t
getNumberOfLandmarks(Environment * env)
{
	return env->landmarkNumber;
}

void
addLandmark(Environment * env, float position)
{
	if (env->landmarkNumber >= kEnvironmentConstantMaxLandmarks - 1)
	{
		return;
	}

	env->landmarkPositions[env->landmarkNumber] = position;
	env->landmarkNumber++;

	return;
}

float
getLandmarkPosition(Environment * env, size_t index)
{
	if (index < env->landmarkNumber)
	{
		return env->landmarkPositions[index];
	}

	return NAN;
}
