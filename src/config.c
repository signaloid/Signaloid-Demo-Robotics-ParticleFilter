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

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "config.h"
#include "environment.h"
#include "model.h"

bool plot = false;

unsigned int maxFilterSteps = 5;

void
printUsage()
{
	printf("Particle filter\n");
	printf("\n");
	printf("    [-h] [-k int] [-l float] [-a float] [-i float] [-e float] [-s float] [-c "
	       "float] [-p]\n");
	printf("\n");
	printf("	-h: print help\n");
	printf("\n");
	printf("	-k int: maximum number of steps (default: %u)\n", maxFilterSteps);
	printf("\n");
	printf("	-l float: add a Landmark at the given position\n");
	printf("\n");
	printf("	-a float: mAximal position (default: %f)\n", positionMax);
	printf("\n");
	printf("	-i float: mInimal position (default: %f)\n", positionMin);
	printf("\n");
	printf("	-e float: Exepcted speed (default: %f)\n", speedExpected);
	printf("\n");
	printf("	-s float: speed Standart deviation (default: %f)\n", speedStd);
	printf("\n");
	printf("	-c float: sensor standart deviation Coefficient (default: %f)\n",
	       sensorStdCoeff);
	printf("\n");
	printf("	-p: output an SVG image instead of text\n");
	printf("\n");
}

float
parseFloat(char * arg, const char * opt)
{
	errno = 0;
	float f = strtof(arg, NULL);
	if (errno != 0)
	{
		perror(opt);
		exit(EXIT_FAILURE);
	}
	return f;
}

int
parseInt(char * arg, const char * opt)
{
	errno = 0;
	int i = (int)strtol(arg, NULL, 10);
	if (errno != 0)
	{
		perror(opt);
		exit(EXIT_FAILURE);
	}
	return i;
}

void
parseArguments(int argc, char * argv[])
{
	int   c;
	long  k;
	float lPosition;
	while ((c = getopt(argc, argv, "hk:l:a:i:e:s:c:p")) != -1)
	{
		switch (c)
		{
		case 'h':
			printUsage();
			exit(EXIT_SUCCESS);
			break;
		case 'k':
			k = parseInt(optarg, "Option '-k'");
			if (k < 0 || k > INT_MAX)
			{
				fprintf(stderr, "Option -k is out of bounds\n");
				exit(EXIT_FAILURE);
			}
			else
			{
				maxFilterSteps = (unsigned int)k;
			}
			break;
		case 'l':
			lPosition = parseFloat(optarg, "Option '-s'");
			addLandmark(lPosition);
			break;
		case 'a':
			positionMax = parseFloat(optarg, "Option '-a'");
			break;
		case 'i':
			positionMin = parseFloat(optarg, "Option '-i'");
			break;
		case 'e':
			speedExpected = parseFloat(optarg, "Option '-e'");
			break;
		case 's':
			speedStd = parseFloat(optarg, "Option '-s'");
			if (speedStd < 0)
			{
				fprintf(stderr,
				        "Option -s needs a positive argument, not %f\n",
				        speedStd);
				exit(EXIT_FAILURE);
			}
			break;
		case 'c':
			sensorStdCoeff = parseFloat(optarg, "Option '-c'");
			if (sensorStdCoeff < 0)
			{
				fprintf(stderr,
				        "Option -c needs a positive argument, not %f\n",
				        speedStd);
				exit(EXIT_FAILURE);
			}
			break;
		case 'p':
			plot = true;
			break;
		default:
			fprintf(stderr, "arguments malformed\n");
			exit(EXIT_FAILURE);
		}
	}
	if (positionMin >= positionMax)
	{
		fprintf(stderr, "minimal position should be smaller than the maximal one\n");
		exit(EXIT_FAILURE);
	}
	if (getNumberOfLandmarks() == 0)
	{
		lPosition = (positionMax + positionMin) / 2;
		fprintf(stderr, "No landmark provided, adding at landmark at %f.\n", lPosition);
		addLandmark(lPosition);
	}
}
