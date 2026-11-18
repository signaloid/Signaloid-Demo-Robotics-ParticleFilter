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
 *	The true state the simulated robot starts from. The demo used to draw both
 *	with `UxHwFloatSample()`, which made every run track a different
 *	realisation: two runs of the same binary, and the same run on two different
 *	execution cores, could not be compared against each other or against a
 *	Monte Carlo reference. They are now scalar parameters, defaulting to the
 *	values in `datatypes.h` except for the start position, and
 *	`--sample-initial-state` restores the drawn start for anyone who wants it.
 *
 *	The robot must not start on a landmark, and must not step onto one either.
 *	`sensorModel()` gives a reading the standard deviation `distance *
 *	sensorStdCoeff`, so at distance zero the evidence is a Dirac and the
 *	Bayes-Laplace update has nothing to weigh: the estimate comes back NaN.
 *	`kSystemConstantsDefaultPosition` is 0.0 and the default landmark sits
 *	midway between `positionMin` and `positionMax`, which is also 0.0, so that
 *	is exactly what a default run used to do once the start stopped being drawn.
 *
 *	-4.5 keeps the robot clear of the landmark at every step of a default run:
 *	the offset is a half-integer and the default speed is 1.0, so no number of
 *	steps lands on 0.0.
 */
#define kParticleFilterDefaultTruePosition  (-4.5)
#define kParticleFilterDefaultTrueSpeed     (kSystemConstantsDefaultSpeed)

/*
 *	One step of a filter run, as recorded for the SVG plot. The kernel does no
 *	I/O -- it has to compile for targets with no operating system under it -- so
 *	a caller that wants to draw the run asks for these values and renders them
 *	itself afterwards, which is what `main.c` does with `plot.c`.
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
 *	@brief	Kernel parameters: the configuration of one filter run. The
 *		command-line parser fills them directly, and `main.c` adds the
 *		landmarks read from the input file, the drawn initial state when
 *		one was asked for, and the trace, all before timing starts. The
 *		kernels do not depend on the demo's command-line arguments. Unlike the
 *		sensor demos, whose parameters are fixed by macros in their own
 *		`kernel.h`, every parameter of this filter is settable from the
 *		command line, so they have to reach the kernel as values rather
 *		than as compile-time constants.
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
	 *	Whether each step's sensor reading is drawn from the sensor model
	 *	with `UxHwFloatSample()` (the demo's original behaviour, a different
	 *	realisation on every run) or is the noiseless distance to the nearest
	 *	landmark. The filter's Bayes-Laplace update uses the noisy sensor
	 *	model either way; this only decides the realised reading it is given.
	 */
	bool useSampledReadings;
	/*
	 *	Where to record the run, or NULL to record nothing. This is the one
	 *	output in this struct: keeping it here, rather than adding a
	 *	parameter to every kernel entry point, leaves those entry points the
	 *	same shape as the other demos'.
	 */
	ParticleFilterTrace * trace;
} KernelParameters;

/**
 *	@brief	Fills the environment and the model from the parameters. With no
 *		landmark configured, places one midway between `positionMin` and
 *		`positionMax`. Shared by the UxHw and Monte Carlo kernels.
 *
 *	@param	parameters	: The filter's parameters.
 *	@param	env		: The environment to fill. Must be zero-initialised.
 *	@param	model		: The model to fill.
 */
void
ParticleFilterSetUpEnvironmentAndModel(
	const KernelParameters *    parameters,
	Environment *               env,
	Model *                     model);

/**
 *	@brief	Distance from a position to the nearest landmark, with no sensor
 *		noise added: the reading a perfect sensor would return.
 *
 *	@param	env		: The environment holding the landmark positions.
 *	@param	position	: The position to measure from.
 *	@return	float		: The distance to the closest landmark.
 */
float
nearestLandmarkDistance(Environment * env, float position);

/**
 *	@brief	Particle filter kernel. Runs the 1D filter from its initial position
 *		prior, applying the Bayes-Laplace rule to one sensor reading per
 *		step and advancing both the simulated robot and the filter's
 *		estimate, and writes the resulting posterior to
 *		`outputVariables[kParticleFilterOutputVariableIndexEstimatedPosition]`.
 *
 *	@param	parameters	: The filter's parameters and the true state of the
 *				  simulated robot.
 *	@param	inputVariables	: The array of input variables used in the
 *				  calculation. Reads the initial position prior from
 *				  `inputVariables[kParticleFilterInputVariableIndexInitialPosition]`.
 *	@param	outputVariables	: An array of output variables. Writes the result to
 *				  `outputVariables[kParticleFilterOutputVariableIndexEstimatedPosition]`.
 *	@return	double		: Returns the distributional value calculated.
 */
double
ParticleFilter_calculateOutput(
	const KernelParameters *    parameters,
	double *                    inputVariables,
	double *                    outputVariables);

/**
 *	@brief	Populate
 *		`inputVariables[kParticleFilterInputVariableIndexInitialPosition]`
 *		with the filter's prior over the robot's position, which is
 *		`UxHwDoubleUniformDist(positionMin, positionMax)`: the robot knows
 *		nothing about where it starts, so every position in range is
 *		equally likely.
 *
 *		This helper is shared by both the UxHw-mode and Monte Carlo-mode
 *		kernels (`ParticleFilterUxHw` in `particle-filter-uxhw.c` and
 *		`ParticleFilterMonteCarlo` in `particle-filter-monte-carlo.c`): in
 *		both cases the input is generated by the same
 *		`UxHwDoubleUniformDist` call, which either returns a full
 *		distribution (real Signaloid hardware) or a single pseudo-random
 *		sample per call (the native compat build), depending on the
 *		execution backend.
 *
 *	@param	parameters	: The filter's parameters; reads `positionMin` and
 *				  `positionMax`.
 *	@param	inputVariables	: Array of size `kParticleFilterInputVariableIndexMax` to fill.
 */
void
ParticleFilterSetInputVariablesViaUxHwCall(
	const KernelParameters *    parameters,
	double *                    inputVariables);

/**
 *	@brief	UxHw-mode calculation kernel. Sizes and owns the `inputVariables`
 *		array (dimensioned by `kParticleFilterInputVariableIndexMax`) and
 *		dispatches to the purely computational `ParticleFilterUxHw` (in
 *		`particle-filter-uxhw.c`).
 *
 *	@param	parameters		: The filter's parameters and the true state
 *					  of the simulated robot.
 *	@param	outputVariables		: Array of size `kParticleFilterOutputVariableIndexMax` to fill.
 *	@param	monteCarloOutputSamples	: Single-element array for the distributional result.
 *	@return	double			: Returns the estimated position.
 */
double
calculateOutputUxHw(
	const KernelParameters *    parameters,
	double *                    outputVariables,
	double *                    monteCarloOutputSamples);

/**
 *	@brief	Monte Carlo calculation kernel. Sizes and owns the `inputVariables`
 *		array (dimensioned by `kParticleFilterInputVariableIndexMax`) and
 *		dispatches to the purely computational `ParticleFilterMonteCarlo`
 *		(in `particle-filter-monte-carlo.c`).
 *
 *	@param	parameters			: The filter's parameters and the
 *						  true state of the simulated robot.
 *	@param	numberOfMonteCarloIterations	: Number of Monte Carlo iterations to run.
 *	@param	outputVariables			: Array of size `kParticleFilterOutputVariableIndexMax` to fill.
 *	@param	monteCarloOutputSamples		: Array of `numberOfMonteCarloIterations` doubles, filled with samples.
 *	@return	double				: Returns the last particle after the final step.
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
 *	@brief	Auxiliary function acting as a wrapper for calling the `sensorModel`
 *		function as an evidence function through `UxHwFloatBayesLaplace`.
 *
 *	@param	args	: A pointer to the arguments `sensorModel` requires,
 *			  typecast to a `void *`.
 *	@param	x	: The float argument of the evidence function.
 *	@return	float	: Returns the value returned by the `sensorModel` function.
 */
float
sensorModelEvidence(void * args, float x);
