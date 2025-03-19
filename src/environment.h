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

#pragma once

#include <stdlib.h>
#include "datatypes.h"

/**
 *	@brief	Gets the Number of landmarks in the environment
 *
 *	@param	env	: The pointer to the environment
 *	@return size_t	: Returns the number of landmarks in the environment
 */
size_t	getNumberOfLandmarks(Environment *  env);

/**
 *	@brief	Add new landmarks to the environment
 *
 *	@param	env		: The pointer to the environment
 *	@param	position	: The position of the new landmark to add
 */
void	addLandmark(Environment *  env, float position);

/**
 *	@brief	Get a landmark's position (what the sensor picks up)
 *
 *	@param	env	: The pointer to the environment
 *	@param	index	: The index of the landmark
 *	@return float	: Returns the position of the specified landmark
 */
float	getLandmarkPosition(Environment *  env, size_t index);
