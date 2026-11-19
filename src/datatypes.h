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

#include <stddef.h>

#define kEnvironmentConstantDefaultPositionMin  (-10.0)
#define kEnvironmentConstantDefaultPositionMax  (10.0)

typedef enum
{
	kEnvironmentConstantMaxLandmarks            = 128,
	kEnvironmentConstantDefaultLandmarkNumber   = 0,
} EnvironmentConstant;

typedef struct Environment
{
	/*
	 *	Minimal possible position
	 */
	float   positionMin;
	/*
	 *	Maximal possible position
	 */
	float   positionMax;
	size_t  landmarkNumber;
	float   landmarkPositions[kEnvironmentConstantMaxLandmarks];
} Environment;

#define kModelConstantDefaultSpeedExpected  (1.0)
#define kModelConstantDefaultSpeedStd       (0.5)
#define kModelConstantDefaultSensorStdCoeff (0.5)

typedef struct Model
{
	/*
	 *	The expected value for the speed
	 */
	float   speedExpected;
	/*
	 *	The standard deviation for the speed
	 */
	float   speedStd;
	/*
	 *	The standard deviation of the sensor noise depends on the position.
	 *	There is more noise on the readings of landmarks further away.
	 */
	float sensorStdCoeff;
} Model;

#define kSystemConstantsDefaultPosition (0.0)
#define kSystemConstantsDefaultSpeed    (1.0)

typedef struct System
{
	/*
	 *	State of the system (particle)
	 */
	float   position;
	/*
	 *	The actual speed.
	 */
	float   speed;
} System;

typedef enum
{
	kPlotConstantsMaxFilterSteps = 5,
} PlotConstants;
