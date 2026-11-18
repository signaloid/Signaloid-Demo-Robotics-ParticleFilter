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

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <uxhw.h>

#include "common.h"
#include "environment.h"
#include "kernel.h"
#include "model.h"
#include "particle-filter-monte-carlo.h"
#include "system.h"

#define kParticleFilterInverseSqrtTwoPi (0.3989422804014327)

/**
 *	@brief	Density at `reading` of the sensor model's output for a robot at
 *		`position`. This is the density `UxHwFloatBayesLaplace()` weighs the
 *		prior with, in closed form.
 *
 *		`sensorModel()` returns the smallest of one noisy distance per
 *		landmark, each Gaussian with mean `d` and standard deviation
 *		`d * sensorStdCoeff`. The density of that minimum of independent
 *		Gaussians is the sum over landmarks `i` of `pdf_i(reading)` times the
 *		product over the other landmarks `j` of `1 - cdf_j(reading)`.
 *
 *	@param	model		: The sensor model.
 *	@param	env		: The environment holding the landmarks.
 *	@param	position	: The position the robot is assumed to be at.
 *	@param	reading		: The sensor reading.
 *	@return	double		: The likelihood of the reading.
 */
static double
sensorLikelihood(Model * model, Environment * env, float position, float reading)
{
	const size_t    landmarkCount   = getNumberOfLandmarks(env);
	double          density         = 0.0;

	for (size_t ii = 0; ii < landmarkCount; ii++)
	{
		const double    mean        = fabs(getLandmarkPosition(env, ii) - position);
		const double    sigma       = mean * model->sensorStdCoeff;
		double          termDensity = 0.0;

		/*
		 *	A zero sigma is a Dirac, with no density to weigh.
		 */
		if (sigma > 0.0)
		{
			const double z = (reading - mean) / sigma;

			termDensity = kParticleFilterInverseSqrtTwoPi * exp(-0.5 * z * z) / sigma;
		}

		for (size_t jj = 0; jj < landmarkCount; jj++)
		{
			if (jj != ii)
			{
				const double    otherMean   = fabs(getLandmarkPosition(env, jj) - position);
				const double    otherSigma  = otherMean * model->sensorStdCoeff;
				double          survival    = (reading < otherMean) ? 1.0 : 0.0;

				if (otherSigma > 0.0)
				{
					survival = 0.5 * erfc((reading - otherMean) / otherSigma * M_SQRT1_2);
				}

				termDensity *= survival;
			}
		}

		density += termDensity;
	}

	return density;
}

/**
 *	@brief	Index of the first entry of the ascending `cumulativeWeights` that
 *		is greater than `target`.
 *
 *	@param	cumulativeWeights	: Running sums of the weights.
 *	@param	count			: Number of entries.
 *	@param	target			: The value to look up.
 *	@return	size_t			: The index found, at most `count - 1`.
 */
static size_t
findParticleIndex(const double * cumulativeWeights, size_t count, double target)
{
	size_t  low     = 0;
	size_t  high    = count - 1;

	while (low < high)
	{
		const size_t middle = low + (high - low) / 2;

		if (cumulativeWeights[middle] > target)
		{
			high = middle;
		}
		else
		{
			low = middle + 1;
		}
	}

	return low;
}

/**
 *	@brief	Mean of the particles, for the SVG trace.
 */
static float
particleMean(const double * particles, size_t count)
{
	double sum = 0.0;

	for (size_t ii = 0; ii < count; ii++)
	{
		sum += particles[ii];
	}

	return (float) (sum / (double) count);
}

double
ParticleFilterMonteCarlo(
	size_t                      numberOfMonteCarloIterations,
	const KernelParameters *    parameters,
	double *                    inputVariables,
	double *                    outputVariables,
	double *                    monteCarloOutputSamples)
{
	Environment env     = { 0 };
	Model       model   = { 0 };
	System      system  = { 0 };
	size_t      steps   = 0;

	if (numberOfMonteCarloIterations == 0)
	{
		return 0.0;
	}

	ParticleFilterSetUpEnvironmentAndModel(parameters, &env, &model);

	system.position = parameters->truePosition;
	system.speed    = parameters->trueSpeed;

	/*
	 *	One filter with a particle per requested sample. The particles
	 *	live in `monteCarloOutputSamples`, so that they are the output.
	 */
	double *    particles           = monteCarloOutputSamples;
	double *    cumulativeWeights   = (double *) checkedMalloc(numberOfMonteCarloIterations * sizeof(double), __FILE__, __LINE__);
	double *    resampled           = (double *) checkedMalloc(numberOfMonteCarloIterations * sizeof(double), __FILE__, __LINE__);

	/*
	 *	Draw the particles from the same prior the UxHw path uses.
	 */
	for (size_t ii = 0; ii < numberOfMonteCarloIterations; ii++)
	{
		ParticleFilterSetInputVariablesViaUxHwCall(parameters, inputVariables);

		particles[ii] = inputVariables[kParticleFilterInputVariableIndexInitialPosition];
	}

	if (parameters->trace != NULL)
	{
		parameters->trace->count = 0;
	}

	while ((system.position > env.positionMin) &&
	       (system.position < env.positionMax) &&
	       (steps < parameters->maxNumberOfSteps))
	{
		const float priorEstimatedPosition = particleMean(particles, numberOfMonteCarloIterations);
		float       sensorReading;
		double      totalWeight = 0.0;

		if (parameters->useSampledReadings)
		{
			sensorReading = sense(&system, &model, &env);
		}
		else
		{
			sensorReading = nearestLandmarkDistance(&env, system.position);
		}

		/*
		 *	Weight each particle by the sensor likelihood of the reading,
		 *	the Bayes update.
		 */
		for (size_t ii = 0; ii < numberOfMonteCarloIterations; ii++)
		{
			totalWeight             += sensorLikelihood(&model, &env, (float) particles[ii], sensorReading);
			cumulativeWeights[ii]   = totalWeight;
		}

		/*
		 *	Resample in proportion to the weights. If every weight is zero
		 *	the reading rules out all particles, so keep them as they are.
		 */
		if ((totalWeight > 0.0) && isfinite(totalWeight))
		{
			for (size_t ii = 0; ii < numberOfMonteCarloIterations; ii++)
			{
				const double target = UxHwDoubleUniformDist(0.0, totalWeight);

				resampled[ii] = particles[findParticleIndex(cumulativeWeights, numberOfMonteCarloIterations, target)];
			}

			for (size_t ii = 0; ii < numberOfMonteCarloIterations; ii++)
			{
				particles[ii] = resampled[ii];
			}
		}

		if ((parameters->trace != NULL) &&
		    (parameters->trace->count < parameters->trace->capacity))
		{
			ParticleFilterTraceStep * traceStep = &parameters->trace->steps[parameters->trace->count];

			traceStep->truePosition                 = system.position;
			traceStep->priorEstimatedPosition       = priorEstimatedPosition;
			traceStep->posteriorEstimatedPosition   = particleMean(particles, numberOfMonteCarloIterations);
			traceStep->sensorReading                = sensorReading;
			parameters->trace->count++;
		}

		move(&system);

		for (size_t ii = 0; ii < numberOfMonteCarloIterations; ii++)
		{
			particles[ii] = moveModel(&model, (float) particles[ii]);
		}

		steps++;
	}

	free(cumulativeWeights);
	free(resampled);

	outputVariables[kParticleFilterOutputVariableIndexEstimatedPosition] = particles[numberOfMonteCarloIterations - 1];

	return outputVariables[kParticleFilterOutputVariableIndexEstimatedPosition];
}
