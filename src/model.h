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
#pragma once

/*
 *	The expected value for the speed
 */
extern float speedExpected;

/*
 *	The standard deviation for the speed
 */
extern float speedStd;

/*
 *	The standard deviation of the sensor noise depends on the position.
 *	There is more noise on the readings of landmarks further away.
 */
extern float sensorStdCoeff;

/*
 *	Returns the distribution of the system's speed.
 */
float
speedDistribution();

/*
 *	Sensor model for the Bayes-Laplace rule.
 *	The sensor model returns: N(position, sensorStdCoeff*position).
 */
float
sensorModel(float position);

/*
 *	Model of the motion used by the filter
 */
float
moveModel(float position);
