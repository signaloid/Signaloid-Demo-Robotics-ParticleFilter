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

#include <math.h>
#include <stddef.h>
#include <uxhw.h>

#include "environment.h"
#include "model.h"
#include "plot.h"
#include "system.h"

typedef enum
{
	nbBuckets           = 64,
	gradientSteps       = 32,
	svgWidth            = 800,
	svgHeader           = 150,
	svgFrameHeight      = 275,
	textOffsetX         = 10,
	textOffsetY         = 25,
	carWidth            = 40,
	carHeight           = 20,
	landmarkRadius      = 5,
	distributionHeight  = 50,
	distributionOffset  = 25,
	sensorAlphaRatio    = 2,
	buffer              = 50
} Layout;

/**
 *	@brief	Calculates the relative X-coordinate of a position in the SVG canvas.
 *
 *	@param env		: A pointer to the environment.
 *	@param position		: The position to calculate.
 *	@return float		: The relative X-coordinate.
 */
static float
positionToX(Environment * env, float position)
{
	return (position - env->positionMin) / (env->positionMax - env->positionMin) * svgWidth;
}

/**
 *	@brief	Generates an svg text element, by printing in a file.
 *
 *	@param	stream	: The file pointer where the element will be printed.
 * 	@param	x 	: The x-coordinate of the element.
 *	@param	y	: The y-coordinate of the element.
 *	@param	txt	: The txt field of the element.
 *	@param	class	: The class field of the element.
 */
static void
text(FILE * stream, float x, float y, const char * txt, const char *  class)
{
	fprintf(
		stream,
		"<text x=\"%"SignaloidParticleModifier"f\""
		" y=\"%"SignaloidParticleModifier"f\""
		" class=\"%s\">%s</text>\n",
		x,
		y,
		class,
		txt
	);

	return;
}

/**
 *	@brief	Generates an svg line element, by printing in a file.
 *
 *	@param	stream	: The file pointer where the element will be printed.
 *	@param	x1	: The x-coordinate of the first point of the line.
 *	@param	y1	: The y-coordinate of the first point of the line.
 *	@param	x2	: The x-coordinate of the second point of the line.
 *	@param	y2	: The y-coordinate of the second point of the line.
 *	@param	class	: The class field of the element.
 */
static void
line(FILE * stream, float x1, float y1, float x2, float y2, const char *  class)
{
	fprintf(
		stream,
		"<line x1=\"%"SignaloidParticleModifier"f\""
		" y1=\"%"SignaloidParticleModifier"f\""
		" x2=\"%"SignaloidParticleModifier"f\""
		" y2=\"%"SignaloidParticleModifier"f\" class=\"%s\"/>\n",
		x1,
		y1,
		x2,
		y2,
		class
	);

	return;
}

/**
 *	@brief	Generates an svg rect element, by printing in a file.
 *
 *	@param	stream	: The file pointer where the element will be printed.
 *	@param	x 	: The x-coordinate of the rectangle.
 *	@param	y	: The y-coordinate of the rectangle.
 *	@param	width	: The width of the rectangle.
 *	@param	height	: The height of the rectangle.
 *	@param	class	: The class field of the element.
 */
static void
rect(FILE * stream, float x, float y, float width, float height, const char *  class)
{
	fprintf(
		stream,
		"<rect x=\"%"SignaloidParticleModifier"f\""
		" y=\"%"SignaloidParticleModifier"f\""
		" width=\"%"SignaloidParticleModifier"f\""
		" height=\"%"SignaloidParticleModifier"f\" class=\"%s\"/>\n",
		x,
		y,
		width,
		height,
		class
	);

	return;
}

/**
 *	@brief	Generates an svg circle element, by printing in a file.
 *
 *	@param	stream	: The file pointer where the element will be printed.
 *	@param	cx	: The x-coordinate of the center of the circle.
 *	@param	cy	: The y-coordinate of the center of the circle.
 *	@param	r	: The radius of the circle.
 *	@param	class	: The class field of the element.
 */
static void
circle(FILE * stream, float cx, float cy, float r, const char *  class)
{
	fprintf(
		stream,
		"<circle cx=\"%"SignaloidParticleModifier"f\""
		" cy=\"%"SignaloidParticleModifier"f\""
		" r=\"%"SignaloidParticleModifier"f\" class=\"%s\"/>\n",
		cx,
		cy,
		r,
		class
	);

	return;
}

/**
 *	@brief	Calculates the position of the start of the bucket with index `index`.
 *
 *	@param	env	: The pointer to the environment.
 *	@param	index	: The index of the bucket.
 *	@return	float	: Returns the position of the bucket start.
 */
static float
bucketStart(Environment * env, unsigned int index)
{
	return env->positionMin + (env->positionMax - env->positionMin) / nbBuckets * index;
}

/**
 *	@brief	Calculates the position of the end of the bucket with index `index`.
 *
 *	@param	env	: The pointer to the environment.
 *	@param	index	: The index of the bucket.
 *	@return	float	: Returns the position of the bucket end.
 */
static float
bucketEnd(Environment * env, unsigned int index)
{
	return bucketStart(env, index + 1);
}

/**
 *	@brief	Plots a histogram for a distribution as an svg and writes it to a file.
 *
 *	@param	stream	: The file pointer where the element will be printed.
 *	@param	env	: The pointer to the environment.
 *	@param	x	: The x-coordinate of the plot.
 *	@param	y	: The y-coordinate of the plot.
 *	@param	dist	: The distributional value to plot.
 *	@param	class	: The class field of the element.
 */
static void
distribution(
	FILE *          stream,
	Environment *   env,
	float           x,
	float           y,
	float           dist,
	const char *    class)
{
	float   boundaries[nbBuckets + 1];
	float   density[nbBuckets];
	float   maxDensity = 0;
	float   mass;
	float   plotStart;
	float   plotEnd;
	float   plotHeight;
	size_t  i;

	boundaries[0] = bucketStart(env, 0);

	for (i = 0; i < nbBuckets; i++)
	{
		boundaries[i + 1]   = bucketEnd(env, i);
		mass                = UxHwFloatProbabilityGT(dist, boundaries[i]) -
		                      UxHwFloatProbabilityGT(dist, boundaries[i + 1]);
		density[i]  = mass / (env->positionMax - env->positionMin) * nbBuckets;
		maxDensity  = fmax(maxDensity, density[i]);
	}

	for (i = 0; i < nbBuckets; i++)
	{
		plotStart   = x + positionToX(env, boundaries[i]);
		plotEnd     = x + positionToX(env, boundaries[i + 1]);
		plotHeight  = density[i] / maxDensity * distributionHeight;
		rect(stream, plotStart, y - plotHeight, plotEnd - plotStart, plotHeight, class);
	}

	return;
}

/**
 *	@brief	Generates an svg showing the system's image and writes it to a file.
 *
 *	@param	stream		: The file pointer where the element will be printed
 *	@param	env		: The pointer to the environment
 *	@param	x		: The x-coordinate of the image
 *	@param	y		: The y-coordinate of the image
 *	@param	position	: The position of the car
 */
static void
systemSchema(FILE * stream, Environment *  env, float x, float y, float position)
{
	size_t i;

	/*
	 *	show rail
	 */
	line(stream, x, y, x + svgWidth, y, "lineBlack");

	/*
	 *	show the car
	 */
	rect(
		stream,
		x + positionToX(env, position) - carWidth / 2,
		y - carHeight / 2,
		carWidth,
		carHeight,
		"car"
	);

	/*
	 *	show the landmarks
	 */
	for (i = 0; i < getNumberOfLandmarks(env); i++)
	{
		circle(
			stream,
			positionToX(env, getLandmarkPosition(env, i)),
			y + carHeight,
			landmarkRadius,
			"landmark"
		);
	}

	return;
}

void
plotHeader(FILE * stream, Environment *  env, Model *  model, System *  system)
{
	float   systemY;
	float   distributionY;
	float   speedX;

	/*
	 *	SVG header
	 */
	fprintf(
		stream,
		"<?xml version=\"1.0\" standalone=\"no\"?>\n"
		"<svg width=\"%d\" height=\"%d\" version=\"1.2\" "
		"xmlns=\"http://www.w3.org/2000/svg\">\n",
		svgWidth,
		svgHeader + kPlotConstantsMaxFilterSteps * svgFrameHeight
	);
	/*
	 *	style definitions
	 */
	fprintf(
		stream,
		"<style>\n"
		".defaultText { font: 24px sans-serif; }\n"
		".subText { font: 18px sans-serif; }\n"
		".lineBlack { stroke: black; }\n"
		".lineFirebrick { stroke: firebrick; stroke-width: 3px; }\n"
		".car { stroke: black; fill: white; }\n"
		".landmark { stroke: none; fill: black; }\n"
		".prior { stroke: none; fill: #F8DD27; fill-opacity: 0.8; }\n"
		".posterior { stroke: none; fill: #33A333; fill-opacity: 0.8; }\n"
		".frameOdd { stroke: none; fill: white; fill-opacity: 0.4; }\n"
		".frameEven { stroke: none; fill: gray; fill-opacity: 0.4; }\n"
		"</style>\n"
	);
	/*
	 *	show info
	 */
	fprintf(stream, "<g id=\"header\" transform=\"translate(%d,%d)\">\n", 0, 0);
	rect(stream, 0, 0, svgWidth, svgHeader, "frameOdd");
	text(stream, textOffsetX, textOffsetY, "Speed (constant for all steps)", "defaultText");

	/*
	 *	show the system
	 */
	systemY = svgHeader / 4 * 3;
	systemSchema(stream, env, 0, systemY, system->position);
	/*
	 *	speed distribution
	 */
	distributionY = systemY - distributionOffset;
	distribution(stream, env, 0, distributionY, system->position + speedDistribution(model), "posterior");
	/*
	 *	speed actual value
	 */
	speedX = positionToX(env, system->position + system->speed);
	line(
		stream,
		speedX,
		distributionY,
		speedX,
		distributionY - distributionHeight,
		"lineFirebrick"
	);
	fprintf(stream, "</g>\n");

	return;
}

void
plotFrameBeforeBL(
	FILE *          stream,
	Environment *   env,
	System *        system,
	size_t          frame,
	float           priorPosition)
{
	char    str[buffer];
	float   systemY;
	float   distributionY;

	fprintf(
		stream,
		"<g id=\"frame%zu\" transform=\"translate(%d,%zu)\">\n",
		frame,
		0,
		svgHeader + svgFrameHeight * frame
	);

	if (frame % 2 == 0)
	{
		rect(stream, 0, 0, svgWidth, svgFrameHeight, "frameEven");
	}
	else
	{
		rect(stream, 0, 0, svgWidth, svgFrameHeight, "frameOdd");
	}
	snprintf(str, buffer, "Step %zu", frame);
	text(stream, textOffsetX, textOffsetY, str, "defaultText");
	systemY = svgFrameHeight / 4 * 3;
	systemSchema(stream, env, 0, systemY, system->position);

	distributionY = systemY - 2 * distributionOffset - distributionHeight;
	distribution(stream, env, 0, distributionY, priorPosition, "prior");
	text(
		stream,
		2 * textOffsetX,
		distributionY - distributionHeight / 4,
		"Prior knowledge",
		"subText"
	);

	return;
}

void
plotFrameAfterBL(
	FILE *          stream,
	Environment *   env,
	Model *         model,
	System *        system,
	size_t          frame,
	float           estimatedPosition,
	float           sensorReading)
{
	int     i;
	float   systemY;
	float   distributionY;
	float   positionX;
	float   sensorDist;
	float   boundaries[gradientSteps + 1];
	float   density[gradientSteps];
	float   maxDensity;
	float   mass;
	float   offset;
	float   alpha;

	/*
	 *	estimated position
	 */
	systemY         = svgFrameHeight / 4 * 3;
	distributionY   = systemY - distributionOffset;
	distribution(stream, env, 0, distributionY, estimatedPosition, "posterior");
	text(
		stream,
		2 * textOffsetX,
		distributionY - distributionHeight / 4,
		"Estimated position",
		"subText"
	);

	/*
	 *	sensor distribution as gradient
	 */
	fprintf(stream, "<linearGradient id=\"gradient%zu\">\n", frame);
	sensorDist      = sensorModel(model, env, system->position);
	maxDensity      = 0;
	boundaries[0]   = 0;

	for (i = 0; i < gradientSteps; i++)
	{
		boundaries[i + 1]   = ((env->positionMax - env->positionMin) * (i + 1)) / gradientSteps;
		mass                = UxHwFloatProbabilityGT(sensorDist, boundaries[i]) -
		                      UxHwFloatProbabilityGT(sensorDist, boundaries[i + 1]);
		density[i]  = mass / (env->positionMax - env->positionMin) * gradientSteps;
		maxDensity  = fmax(maxDensity, density[i]);
	}

	for (i = gradientSteps - 1; i >= 0; i--)
	{
		offset  = (50.0f * i) / (gradientSteps - 1);
		alpha   = density[i] / maxDensity / sensorAlphaRatio;
		fprintf(
			stream,
			"<stop offset=\"%"SignaloidParticleModifier"f%%\""
			" stop-color=\"blue\" stop-opacity=\"%"SignaloidParticleModifier"f\" />\n",
			50 - offset,
			alpha
		);
	}

	for (i = 1; i < gradientSteps; i++)
	{
		offset  = (50.0f * i) / (gradientSteps - 1);
		alpha   = density[i] / maxDensity / sensorAlphaRatio;
		fprintf(
			stream,
			"<stop offset=\"%"SignaloidParticleModifier"f%%\""
			" stop-color=\"blue\" stop-opacity=\"%"SignaloidParticleModifier"f\" />\n",
			50 + offset,
			alpha
		);
	}

	fprintf(stream, "</linearGradient>\n");
	fprintf(
		stream,
		"<rect x=\"%"SignaloidParticleModifier"f\""
		" y=\"%"SignaloidParticleModifier"f\" width=\"%u\" height=\"%u\" fill=\"url('#gradient%zu')\" "
		"/>\n",
		positionToX(env, system->position) - svgWidth,
		systemY,
		2 * svgWidth,
		carHeight,
		frame
	);

	/*
	 *	Sensor reading
	 */
	positionX = positionToX(env, system->position + sensorReading);
	line(stream, positionX, systemY, positionX, systemY + carHeight, "lineFirebrick");
	positionX = positionToX(env, system->position - sensorReading);
	line(stream, positionX, systemY, positionX, systemY + carHeight, "lineFirebrick");

	fprintf(stream, "</g>\n");

	return;
}

void
plotFooter(FILE * stream)
{
	fprintf(stream, "</svg>\n");

	return;
}
