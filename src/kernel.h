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

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "datatypes.h"

/*
 *	Input Variables:
 *		kParticleFilterInputVariableIndexInitialPosition	: The filter's
 *			prior over the robot's position before the first sensor
 *			reading. This is the one input the kernel takes as a
 *			distribution, set by a single UxHw call in
 *			`ParticleFilterSetInputVariablesViaUxHwCall()`.
 *
 *	The filter's speed distribution is deliberately not an input variable: the
 *	motion model draws a fresh, independent `UxHwFloatGaussDist()` on every step
 *	inside `moveModel()`, so passing one distribution in would make every step
 *	reuse the same draw and change what the filter computes.
 */
typedef enum
{
	kParticleFilterInputVariableIndexInitialPosition = 0,
	kParticleFilterInputVariableIndexMax,
} ParticleFilterInputVariableIndex;

/*
 *	Output Variables:
 *		kParticleFilterOutputVariableIndexEstimatedPosition	: The
 *			filter's posterior over the robot's position after the last
 *			step it executed.
 */
typedef enum
{
	kParticleFilterOutputVariableIndexEstimatedPosition = 0,
	kParticleFilterOutputVariableIndexMax,
} ParticleFilterOutputVariableIndex;

/*
 *	Default true state of the simulated robot. The robot must never sit on a
 *	landmark, or the Bayes-Laplace update returns NaN. The default landmark is
 *	at 0.0 and the default speed is 1.0, so a start of -4.5 never lands on it.
 */
#define kParticleFilterDefaultTruePosition  (-4.5)
#define kParticleFilterDefaultTrueSpeed     (kSystemConstantsDefaultSpeed)

/*
 *	One step of a filter run, recorded so that `main.c` can plot the run.
 */
typedef struct
{
	/*
	 *	The robot's true position at the start of the step.
	 */
	float truePosition;
	/*
	 *	The filter's estimate before and after this step's Bayes-Laplace
	 *	update.
	 */
	float   priorEstimatedPosition;
	float   posteriorEstimatedPosition;
	/*
	 *	The sensor reading the update was given.
	 */
	float   sensorReading;
} ParticleFilterTraceStep;

/*
 *	A trace of a filter run. `capacity` is how many steps `steps` can hold and
 *	`count` is how many the kernel wrote.
 */
typedef struct
{
	size_t                      capacity;
	size_t                      count;
	ParticleFilterTraceStep *   steps;
} ParticleFilterTrace;

/**
 *	@brief	Parameters consumed by the kernels. `getCommandLineArguments()` and
 *		`main()` populate it, so the kernels do not depend on the demo's
 *		`CommandLineArguments`.
 */
typedef struct
{
	/*
	 *	Maximum number of filter steps to execute. The run also stops early
	 *	if the robot leaves [positionMin, positionMax].
	 */
	size_t maxNumberOfSteps;
	/*
	 *	Support of the filter's initial position prior, and the bounds the
	 *	robot travels within.
	 */
	float   positionMin;
	float   positionMax;
	/*
	 *	The motion model: the mean and standard deviation of the speed the
	 *	filter believes the robot moves at.
	 */
	float   speedExpected;
	float   speedStd;
	/*
	 *	The sensor model: the standard deviation of a reading is this
	 *	coefficient times the distance being measured.
	 */
	float sensorStdCoeff;
	/*
	 *	The true state of the simulated robot: where it starts and how fast
	 *	it actually moves. The filter is told neither.
	 */
	float   truePosition;
	float   trueSpeed;
	/*
	 *	Landmark positions the sensor can see. `landmarkCount` of 0 places a
	 *	single landmark midway between positionMin and positionMax, which is
	 *	what the demo does when it is given no input file.
	 */
	size_t  landmarkCount;
	float   landmarkPositions[kEnvironmentConstantMaxLandmarks];
	/*
	 *	If true, each sensor reading is sampled from the sensor model. If
	 *	false, it is the noiseless distance to the nearest landmark.
	 */
	bool useSampledReadings;
	/*
	 *	Where to record the run, or NULL to record nothing.
	 */
	ParticleFilterTrace * trace;
} KernelParameters;

/**
 *	@brief	Fill the environment and the model from the kernel parameters.
 *
 *	@param	parameters	: Kernel parameters.
 *	@param	env		: The environment to fill. Must be zero-initialised.
 *	@param	model		: The model to fill.
 */
void
ParticleFilterSetUpEnvironmentAndModel(
	const KernelParameters *    parameters,
	Environment *               env,
	Model *                     model);

/**
 *	@brief	Distance from a position to the nearest landmark, with no sensor noise.
 *
 *	@param	env		: The environment holding the landmark positions.
 *	@param	position	: The position to measure from.
 *	@return	float		: The distance to the closest landmark.
 */
float
nearestLandmarkDistance(Environment * env, float position);

/**
 *	@brief	Run the particle filter and calculate the estimated position.
 *
 *	@param	parameters	: Kernel parameters.
 *	@param	inputVariables	: The input variables.
 *	@param	outputVariables	: The output variables.
 *	@return	double		: The estimated position.
 */
double
ParticleFilter_calculateOutput(
	const KernelParameters *    parameters,
	double *                    inputVariables,
	double *                    outputVariables);

/**
 *	@brief	Set the initial position prior to a uniform distribution over
 *		[positionMin, positionMax] via a UxHw call.
 *
 *	@param	parameters	: Kernel parameters.
 *	@param	inputVariables	: The input variables.
 */
void
ParticleFilterSetInputVariablesViaUxHwCall(
	const KernelParameters *    parameters,
	double *                    inputVariables);

/**
 *	@brief	UxHw-mode kernel. Runs the filter once on distributional inputs.
 *
 *	@param	parameters		: Kernel parameters.
 *	@param	outputVariables		: The output variables.
 *	@param	monteCarloOutputSamples	: Single-element array for the distributional result.
 *	@return	double			: The estimated position.
 */
double
calculateOutputUxHw(
	const KernelParameters *    parameters,
	double *                    outputVariables,
	double *                    monteCarloOutputSamples);

/**
 *	@brief	Monte Carlo kernel. Runs the filter with one particle per Monte Carlo iteration.
 *
 *	@param	parameters			: Kernel parameters.
 *	@param	numberOfMonteCarloIterations	: Number of Monte Carlo iterations to run.
 *	@param	outputVariables			: The output variables.
 *	@param	monteCarloOutputSamples		: Array of `numberOfMonteCarloIterations` doubles, filled with samples.
 *	@return	double				: The last particle after the final step.
 */
double
calculateOutputMonteCarlo(
	const KernelParameters *    parameters,
	size_t                      numberOfMonteCarloIterations,
	double *                    outputVariables,
	double *                    monteCarloOutputSamples);

/*
 *	The arguments `sensorModel()` needs beyond the position it is evaluated at.
 *	`UxHwFloatBayesLaplace()` passes them through to the evidence function as an
 *	opaque pointer.
 */
typedef struct
{
	Model *         m;
	Environment *   env;
} SensorModelEvidenceArgumentType;

/**
 *	@brief	Wrap `sensorModel()` as an evidence function for `UxHwFloatBayesLaplace()`.
 *
 *	@param	args	: Pointer to a `SensorModelEvidenceArgumentType`.
 *	@param	x	: The position to evaluate the sensor model at.
 *	@return	float	: The value of `sensorModel()`.
 */
float
sensorModelEvidence(void * args, float x);
