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

#include <stddef.h>

#include "kernel.h"

/**
 *	@brief	Monte Carlo calculation kernel. Native builds cannot run
 *		`UxHwFloatBayesLaplace()`, so this runs a classic particle filter
 *		instead. One filter with `numberOfMonteCarloIterations` particles
 *		is drawn from the same uniform prior as the UxHw path (via
 *		`ParticleFilterSetInputVariablesViaUxHwCall`), then each step weighs
 *		every particle by the closed-form sensor likelihood, resamples,
 *		and moves the particles with the motion model. The particles after
 *		the last step are the samples of the posterior, and are written to
 *		`monteCarloOutputSamples`.
 *
 *		The samples are not fully independent, because resampling copies
 *		particles. The last move adds fresh noise to each.
 *
 *		Purely computational: the caller
 *		(`calculateOutputMonteCarlo` in `kernel.c`) sizes
 *		`inputVariables`.
 *
 *	@param	numberOfMonteCarloIterations	: Number of Monte Carlo iterations to run.
 *	@param	parameters			: The filter's parameters and the
 *						  true state of the simulated robot.
 *	@param	inputVariables			: Array to fill via
 *						  `ParticleFilterSetInputVariablesViaUxHwCall`,
 *						  sized by the caller.
 *	@param	outputVariables			: Array to fill with per-output results.
 *	@param	monteCarloOutputSamples		: Array of `numberOfMonteCarloIterations` doubles, filled with samples.
 *	@return	double				: Returns the last particle.
 */
double
ParticleFilterMonteCarlo(
	size_t                      numberOfMonteCarloIterations,
	const KernelParameters *    parameters,
	double *                    inputVariables,
	double *                    outputVariables,
	double *                    monteCarloOutputSamples);
