/*
 *	Authored 2022, Damien Zufferey.
 *
 *	Copyright (c) 2022, Signaloid.
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

#include "config.h"
#include "environment.h"
#include "model.h"
#include "plot.h"
#include "system.h"

typedef enum
{
	nbBuckets = 64,
	gradientSteps = 32,
	svgWidth = 800,
	svgHeader = 150,
	svgFrameHeight = 275,
	textOffsetX = 10,
	textOffsetY = 25,
	carWidth = 40,
	carHeight = 20,
	landmarkRadius = 5,
	distributionHeight = 50,
	distributionOffset = 25,
	sensorAlphaRatio = 2,
	buffer = 50
} Layout;

float
positionToX(float position)
{
	return (position - positionMin) / (positionMax - positionMin) * svgWidth;
}

void
text(FILE * stream, float x, float y, const char * txt, const char * class)
{
	fprintf(stream, "<text x=\"%f\" y=\"%f\" class=\"%s\">%s</text>\n", x, y, class, txt);
}

void
line(FILE * stream, float x1, float y1, float x2, float y2, const char * class)
{
	fprintf(stream,
	        "<line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" class=\"%s\"/>\n",
	        x1,
	        y1,
	        x2,
	        y2,
	        class);
}

void
rect(FILE * stream, float x, float y, float width, float height, const char * class)
{
	fprintf(stream,
	        "<rect x=\"%f\" y=\"%f\" width=\"%f\" height=\"%f\" class=\"%s\"/>\n",
	        x,
	        y,
	        width,
	        height,
	        class);
}

void
circle(FILE * stream, float cx, float cy, float r, const char * class)
{
	fprintf(stream, "<circle cx=\"%f\" cy=\"%f\" r=\"%f\" class=\"%s\"/>\n", cx, cy, r, class);
}

float
bucketStart(unsigned int index)
{
	return positionMin + (positionMax - positionMin) / nbBuckets * index;
}

float
bucketEnd(unsigned int index)
{
	return bucketStart(index + 1);
}

void
distribution(FILE * stream, float x, float y, float dist, const char * class)
{
	float  boundaries[nbBuckets + 1];
	float  density[nbBuckets];
	float  maxDensity = 0;
	float  mass;
	float  plotStart;
	float  plotEnd;
	float  plotHeight;
	size_t i;

	boundaries[0] = bucketStart(0);
	for (i = 0; i < nbBuckets; i++)
	{
		boundaries[i + 1] = bucketEnd(i);
		mass = UxHwFloatProbabilityGT(dist, boundaries[i]) -
		       UxHwFloatProbabilityGT(dist, boundaries[i + 1]);
		density[i] = mass / (positionMax - positionMin) * nbBuckets;
		maxDensity = fmax(maxDensity, density[i]);
	}
	for (i = 0; i < nbBuckets; i++)
	{
		plotStart = x + positionToX(boundaries[i]);
		plotEnd = x + positionToX(boundaries[i + 1]);
		plotHeight = density[i] / maxDensity * distributionHeight;
		rect(stream, plotStart, y - plotHeight, plotEnd - plotStart, plotHeight, class);
	}
}

void
systemSchema(FILE * stream, float x, float y, float position)
{
	size_t i;

	/*
	 *	show rail
	 */
	line(stream, x, y, x + svgWidth, y, "lineBlack");
	/*
	 *	show the car
	 */
	rect(stream,
	     x + positionToX(position) - carWidth / 2,
	     y - carHeight / 2,
	     carWidth,
	     carHeight,
	     "car");
	/*
	 *	show the landmarks
	 */
	for (i = 0; i < getNumberOfLandmarks(); i++)
	{
		circle(stream,
		       positionToX(getLandmarkPosition(i)),
		       y + carHeight,
		       landmarkRadius,
		       "landmark");
	}
}

void
plotHeader(FILE * stream)
{
	float systemY;
	float distributionY;
	float speedX;

	/*
	 *	SVG header
	 */
	fprintf(stream,
	        "<?xml version=\"1.0\" standalone=\"no\"?>\n"
	        "<svg width=\"%d\" height=\"%d\" version=\"1.2\" "
	        "xmlns=\"http://www.w3.org/2000/svg\">\n",
	        svgWidth,
	        svgHeader + maxFilterSteps * svgFrameHeight);
	/*
	 *	style definitions
	 */
	fprintf(stream,
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
	        "</style>\n");
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
	systemSchema(stream, 0, systemY, position);
	/*
	 *	speed distribution
	 */
	distributionY = systemY - distributionOffset;
	distribution(stream, 0, distributionY, position + speedDistribution(), "posterior");
	/*
	 *	speed actual value
	 */
	speedX = positionToX(position + speed);
	line(stream,
	     speedX,
	     distributionY,
	     speedX,
	     distributionY - distributionHeight,
	     "lineFirebrick");
	fprintf(stream, "</g>\n");
}

void
plotFrameBeforeBL(FILE * stream, size_t frame, float priorPosition)
{
	char  str[buffer];
	float systemY;
	float distributionY;

	fprintf(stream,
	        "<g id=\"frame%u\" transform=\"translate(%d,%d)\">\n",
	        frame,
	        0,
	        svgHeader + svgFrameHeight * frame);
	if (frame % 2 == 0)
	{
		rect(stream, 0, 0, svgWidth, svgFrameHeight, "frameEven");
	}
	else
	{
		rect(stream, 0, 0, svgWidth, svgFrameHeight, "frameOdd");
	}
	snprintf(str, buffer, "Step %u", frame);
	text(stream, textOffsetX, textOffsetY, str, "defaultText");
	systemY = svgFrameHeight / 4 * 3;
	systemSchema(stream, 0, systemY, position);

	distributionY = systemY - 2 * distributionOffset - distributionHeight;
	distribution(stream, 0, distributionY, priorPosition, "prior");
	text(stream,
	     2 * textOffsetX,
	     distributionY - distributionHeight / 4,
	     "Prior knowledge",
	     "subText");
}

void
plotFrameAfterBL(FILE * stream, size_t frame, float estimatedPosition, float sensorReading)
{
	int   i;
	float systemY;
	float distributionY;
	float positionX;
	float sensorDist;
	float boundaries[gradientSteps + 1];
	float density[gradientSteps];
	float maxDensity;
	float mass;
	float offset;
	float alpha;

	/*
	 *	estimated position
	 */
	systemY = svgFrameHeight / 4 * 3;
	distributionY = systemY - distributionOffset;
	distribution(stream, 0, distributionY, estimatedPosition, "posterior");
	text(stream,
	     2 * textOffsetX,
	     distributionY - distributionHeight / 4,
	     "Estimated position",
	     "subText");

	/*
	 *	sensor distribution as gradient
	 */
	fprintf(stream, "<linearGradient id=\"gradient%u\">\n", frame);
	sensorDist = sensorModel(position);
	maxDensity = 0;
	boundaries[0] = 0;
	for (i = 0; i < gradientSteps; i++)
	{
		boundaries[i + 1] = ((positionMax - positionMin) * (i + 1)) / gradientSteps;
		mass = UxHwFloatProbabilityGT(sensorDist, boundaries[i]) -
		       UxHwFloatProbabilityGT(sensorDist, boundaries[i + 1]);
		density[i] = mass / (positionMax - positionMin) * gradientSteps;
		maxDensity = fmax(maxDensity, density[i]);
	}
	for (i = gradientSteps - 1; i >= 0; i--)
	{
		offset = (50.0f * i) / (gradientSteps - 1);
		alpha = density[i] / maxDensity / sensorAlphaRatio;
		fprintf(stream,
		        "<stop offset=\"%f%%\" stop-color=\"blue\" stop-opacity=\"%f\" />\n",
		        50 - offset,
		        alpha);
	}
	for (i = 1; i < gradientSteps; i++)
	{
		offset = (50.0f * i) / (gradientSteps - 1);
		alpha = density[i] / maxDensity / sensorAlphaRatio;
		fprintf(stream,
		        "<stop offset=\"%f%%\" stop-color=\"blue\" stop-opacity=\"%f\" />\n",
		        50 + offset,
		        alpha);
	}
	fprintf(stream, "</linearGradient>\n");
	fprintf(stream,
	        "<rect x=\"%f\" y=\"%f\" width=\"%u\" height=\"%u\" fill=\"url('#gradient%u')\" "
	        "/>\n",
	        positionToX(position) - svgWidth,
	        systemY,
	        2 * svgWidth,
	        carHeight,
	        frame);

	/*
	 *	Sensor reading
	 */
	positionX = positionToX(position + sensorReading);
	line(stream, positionX, systemY, positionX, systemY + carHeight, "lineFirebrick");
	positionX = positionToX(position - sensorReading);
	line(stream, positionX, systemY, positionX, systemY + carHeight, "lineFirebrick");

	fprintf(stream, "</g>\n");
}

void
plotFooter(FILE * stream)
{
	fprintf(stream, "</svg>\n");
}
