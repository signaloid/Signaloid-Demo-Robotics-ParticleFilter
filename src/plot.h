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

#include <stdio.h>
#include <stdlib.h>

/**
 *	@brief	Generate and plot the SVG header.
 *
 *	@param	stream	: The file pointer where the element will be printed.
 *	@param	env	: The pointer to the environment.
 *	@param	model	: The pointer to the model.
 *	@param	system	: The pointer to the system.
 */
void
plotHeader(FILE * stream, Environment *  env, Model *  model, System *  system);

/**
 *	@brief	Generate and plot an SVG frame before the Bayesian Localization.
 *
 *	@param	stream		: The file pointer where the element will be printed.
 *	@param	env		: The pointer to the environment.
 *	@param	system		: The pointer to the system.
 *	@param	frame		: The frame index.
 *	@param	priorPosition	: The car's position.
 */
void
plotFrameBeforeBL(
	FILE *          stream,
	Environment *   env,
	System *        system,
	size_t          frame,
	float           priorPosition);

/**
 *	@brief	Generate and plot an SVG frame after the Bayesian Localization.
 *
 *	@param	stream			: The file pointer where the element will be printed.
 *	@param	env			: The pointer to the environment.
 *	@param	model			: The pointer to the model.
 *	@param	system			: The pointer to the system.
 *	@param	frame			: The frame index.
 *	@param	estimatedPosition	: The estimated car's position.
 *	@param	sensorReading		: The sensor reading.
 */
void
plotFrameAfterBL(
	FILE *          stream,
	Environment *   env,
	Model *         model,
	System *        system,
	size_t          frame,
	float           estimatedPosition,
	float           sensorReading);

/**
 *	@brief	Generate and plot the SVG footer.
 *
 *	@param	stream	: The file pointer where the element will be printed.
 */
void
plotFooter(FILE * stream);
