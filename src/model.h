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

#pragma once

#include "datatypes.h"

/**
 *	@brief	Returns the distribution of the system's speed.
 *
 *	@param	model	: The pointer to the model
 *	@return float	: The speed distribution
 */
float
speedDistribution(Model * model);

/**
 *	@brief	Sensor model for the Bayes-Laplace rule.
 *
 *
 *	@param	model		: The pointer to the model
 *	@param	position	: The current position
 *	@return float		: Returns N(position, sensorStdCoeff * position).
 */
float
sensorModel(Model * model, Environment *  env, float position);

/**
 *	@brief	Model of the motion used by the filter
 *
 *	@param	model		: The pointer to the model
 *	@param	position	: The current position
 *	@return float		: Returns the new position after applying the speedDistribution
 */
float
moveModel(Model * model, float position);
