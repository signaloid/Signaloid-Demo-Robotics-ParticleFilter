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

#include "environment.h"
#include "assert.h"

typedef enum
{
	kMaxLandmarks = 128,
} EnvironmentConstants;

float positionMin = -10;

float positionMax = 10;

size_t landmarkNumber = 0;

/*
 *	To keep the code simple, statically allocate the memory for kMaxLandmarks
 */
float landmarkPositions[kMaxLandmarks];

size_t
getNumberOfLandmarks()
{
	return landmarkNumber;
}

void
addLandmark(float position)
{
	assert(landmarkNumber < kMaxLandmarks - 1);
	landmarkPositions[landmarkNumber] = position;
	landmarkNumber++;
}

float
getLandmarkPosition(size_t index)
{
	assert(index < landmarkNumber);
	return landmarkPositions[index];
}
