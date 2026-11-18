/*
 *	Copyright (c) 2026, Signaloid.
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

#include "kernel.h"

/**
 *	@brief	UxHw-mode calculation kernel. Performs a single distributional
 *		evaluation of the particle filter: sets the initial position prior
 *		once via `ParticleFilterSetInputVariablesViaUxHwCall` and calls
 *		`ParticleFilter_calculateOutput` once. Writes the result to
 *		`outputVariables[0]` and to `monteCarloOutputSamples[0]`.
 *
 *		Purely computational: the caller
 *		(`calculateOutputUxHw` in `kernel.c`) sizes
 *		`inputVariables`.
 *
 *	@param	parameters		: The filter's parameters and the true state
 *					  of the simulated robot.
 *	@param	inputVariables		: Array to fill via
 *					  `ParticleFilterSetInputVariablesViaUxHwCall`,
 *					  sized by the caller.
 *	@param	outputVariables		: Array to fill with per-output results.
 *	@param	monteCarloOutputSamples	: Single-element array for the distributional result.
 *	@return	double			: Returns the estimated position.
 */
double
ParticleFilterUxHw(
	const KernelParameters *    parameters,
	double *                    inputVariables,
	double *                    outputVariables,
	double *                    monteCarloOutputSamples);
