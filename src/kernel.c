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
#include <uxhw.h>

#include "environment.h"
#include "kernel.h"
#include "model.h"
#include "particle-filter-monte-carlo.h"
#include "particle-filter-uxhw.h"
#include "system.h"

/**
 *	@brief	Distance from a position to the nearest landmark, with no sensor
 *		noise added.
 *
 *		This is `sensorModel()` without its `UxHwFloatGaussDist()` call: the
 *		reading a perfect sensor would return. It is what the filter is fed
 *		when `useSampledReadings` is false, so that a run is reproducible.
 *
 *		Shared with the Monte Carlo kernel, declared in `kernel.h`.
 *
 *	@param	env		: The environment holding the landmark positions.
 *	@param	position	: The position to measure from.
 *	@return	float		: The distance to the closest landmark.
 */
float
nearestLandmarkDistance(Environment * env, float position)
{
	float closest = INFINITY;

	for (size_t i = 0; i < getNumberOfLandmarks(env); i++)
	{
		closest = fmin(closest, fabs(getLandmarkPosition(env, i) - position));
	}

	return closest;
}

float
sensorModelEvidence(void * args, float x)
{
	Model *         m;
	Environment *   env;

	m   = ((SensorModelEvidenceArgumentType *) args)->m;
	env = ((SensorModelEvidenceArgumentType *) args)->env;

	return sensorModel(m, env, x);
}

void
ParticleFilterSetUpEnvironmentAndModel(
	const KernelParameters *    parameters,
	Environment *               env,
	Model *                     model)
{
	model->speedExpected    = parameters->speedExpected;
	model->speedStd         = parameters->speedStd;
	model->sensorStdCoeff   = parameters->sensorStdCoeff;

	env->positionMin    = parameters->positionMin;
	env->positionMax    = parameters->positionMax;

	for (size_t i = 0; i < parameters->landmarkCount; i++)
	{
		addLandmark(env, parameters->landmarkPositions[i]);
	}

	/*
	 *	If no landmark was configured, place one in the middle of the allowed
	 *	positions, as the demo does when it is given no input file.
	 */
	if (getNumberOfLandmarks(env) == 0)
	{
		addLandmark(env, (env->positionMin + env->positionMax) / 2);
	}

	return;
}

double
ParticleFilter_calculateOutput(
	const KernelParameters *    parameters,
	double *                    inputVariables,
	double *                    outputVariables)
{
	Environment                     env     = { 0 };
	Model                           model   = { 0 };
	System                          system  = { 0 };
	SensorModelEvidenceArgumentType sensorModelArgs;
	float                           estimatedPosition;
	size_t                          steps = 0;

	ParticleFilterSetUpEnvironmentAndModel(parameters, &env, &model);

	/*
	 *	The filter's state. The prior is the one input variable: the robot
	 *	has no idea where it starts.
	 */
	estimatedPosition = (float) inputVariables[kParticleFilterInputVariableIndexInitialPosition];

	/*
	 *	The true state, which the filter is not told.
	 */
	system.position = parameters->truePosition;
	system.speed    = parameters->trueSpeed;

	sensorModelArgs.m   = &model;
	sensorModelArgs.env = &env;

	/*
	 *	Main computational loop. Each step takes a sensor reading, folds it
	 *	into the estimate with the Bayes-Laplace rule, then advances both the
	 *	robot and the estimate by the motion model.
	 */
	if (parameters->trace != NULL)
	{
		parameters->trace->count = 0;
	}

	while ((system.position > env.positionMin) &&
	       (system.position < env.positionMax) &&
	       (steps < parameters->maxNumberOfSteps))
	{
		float   sensorReading;
		float   priorEstimatedPosition = estimatedPosition;

		if (parameters->useSampledReadings)
		{
			sensorReading = sense(&system, &model, &env);
		}
		else
		{
			sensorReading = nearestLandmarkDistance(&env, system.position);
		}

		estimatedPosition = UxHwFloatBayesLaplace(
			&sensorModelEvidence,
			(void *) (&sensorModelArgs),
			estimatedPosition,
			sensorReading,
			1
		);

		if ((parameters->trace != NULL) &&
		    (parameters->trace->count < parameters->trace->capacity))
		{
			ParticleFilterTraceStep * traceStep = &parameters->trace->steps[parameters->trace->count];

			traceStep->truePosition                 = system.position;
			traceStep->priorEstimatedPosition       = priorEstimatedPosition;
			traceStep->posteriorEstimatedPosition   = estimatedPosition;
			traceStep->sensorReading                = sensorReading;
			parameters->trace->count++;
		}

		move(&system);
		estimatedPosition = moveModel(&model, estimatedPosition);

		steps++;
	}

	/*
	 *	The filter computes in single precision, as the demo always has. The
	 *	kernel interface is double, like every other demo's, so the estimate
	 *	is widened here at the boundary.
	 */
	outputVariables[kParticleFilterOutputVariableIndexEstimatedPosition] = (double) estimatedPosition;

	return outputVariables[kParticleFilterOutputVariableIndexEstimatedPosition];
}

void
ParticleFilterSetInputVariablesViaUxHwCall(
	const KernelParameters *    parameters,
	double *                    inputVariables)
{
	inputVariables[kParticleFilterInputVariableIndexInitialPosition] = UxHwDoubleUniformDist(
		parameters->positionMin,
		parameters->positionMax
	);

	return;
}

double
calculateOutputUxHw(
	const KernelParameters *    parameters,
	double *                    outputVariables,
	double *                    monteCarloOutputSamples)
{
	double inputVariables[kParticleFilterInputVariableIndexMax];

	return ParticleFilterUxHw(
		parameters,
		inputVariables,
		outputVariables,
		monteCarloOutputSamples
	);
}

double
calculateOutputMonteCarlo(
	const KernelParameters *    parameters,
	size_t                      numberOfMonteCarloIterations,
	double *                    outputVariables,
	double *                    monteCarloOutputSamples)
{
	double inputVariables[kParticleFilterInputVariableIndexMax];

	return ParticleFilterMonteCarlo(
		numberOfMonteCarloIterations,
		parameters,
		inputVariables,
		outputVariables,
		monteCarloOutputSamples
	);
}
