[<img src="https://assets.signaloid.io/add-to-signaloid-cloud-logo-dark-v6.png#gh-dark-mode-only" alt="[Add to signaloid.io]" height="30">](https://signaloid.io/repositories?connect=https://github.com/signaloid/Signaloid-Demo-Robotics-ParticleFilter#gh-dark-mode-only)
[<img src="https://assets.signaloid.io/add-to-signaloid-cloud-logo-light-v6.png#gh-light-mode-only" alt="[Add to signaloid.io]" height="30">](https://signaloid.io/repositories?connect=https://github.com/signaloid/Signaloid-Demo-Robotics-ParticleFilter#gh-light-mode-only)

# Particle Filter

This repository contains a 1D [Particle Filter](https://en.wikipedia.org/wiki/Particle_filter) implemented using Signaloid's uncertainty tracking technology.

A particle filter is an algorithm to solve the [filtering, or state estimation, problem](https://en.wikipedia.org/wiki/Filtering_problem_%28stochastic_processes%29).
The goal is to estimate the state of an evolving system from sensor observations.
The challenges are that the system is usually not fully observable and the sensors are subject to noise.
Robot localization is an instance of the state estimation problem.

The robot localization problem corresponds to a robot trying to figure out its location in a known environment.
The robot does not have a sensor, such as a GPS, that directly give it's location.
Instead, it is equipped with radars or cameras which give information about the surroundings.
Using this information and the knowledge of the environment, the robot tries to determine its location.

A particle filter approximates the distribution of the state using a collection of _particles_.
Each particle is a concrete state, e.g., a robot's position.
The motion of the robot is simulated on each particle and each particle is evaluated against the sensor observations.
In the evaluation, the particles that are more likely given the sensor readings get more weights.
The particles get resampled proportionally to the weights and the process repeats.


Signaloid's uncertainty tracking helps in two key aspects of the filter.
1. The particles are implicitly represented using the processor uncertainty tracking.
2. Bayesian reasoning in [libUncertain](https://libuncertain.signaloid.io/) does the majority's of the filter's work, taking care of the comparison against the sensor readings and resampling.

## 1 Degree of Freedom Robotic System with a Distance Senor

The example is a 1 degree of freedom mobile robot.
This, for instance, can be a robot moving on a rail or a gantry.

The robot has a distance sensor that can detect nearby objects.
We call the objects that are detected _landmarks_ and we know their position.
The sensor returns the distance of the closest landmark but does not indicate if it is in front or behind the robot.

Here is a visual representation of the system.

![Model of the system](resources/system.svg)

The robot does not know its initial position and moves at a constant speed.
It does not know the exact speed, only the distribution from which the speed is drawn.
Furthermore, the measurements of the sensor are subject to additive Gaussian noise.


## Program Output


Running this program on [signaloid.io](https://signaloid.io/) produces a textual output by default.
It is possible to produce an SVG image instead using the option `-p`.

Here an example output image with some annotation to explain all the elements.

First, we show the actual speed of the robot and the speed distribution used by the filter.
Then, we show each time step.

![Sample output](resources/sample_output.svg)

The output also shows the mode of distribution.
The mode is the maximum of the density.
This can be helpful when dealing with multimodal distributions.

## How Signaloid's Technology Helps

A particle filter keeps particles to approximate a distribution of the possible state.
As the Signaloid C0 processor can natively track distributions, we do not need to explicitly track particles.
Signaloid C0 is taking care of all that for us.
For instance, in [`main.c`](src/main.c), the filter is initialized with
```c
estimatedPosition = libUncertainFloatUniformDist(positionMin, positionMax);
```
This tells the processor that we have no information about the system and that every position is equally likely.

In addition, the Signaloid C0 uncertainty representations are a powerful tool to concisely model the sensor.
In [`model.c`](src/model.c), the `sensorModel` function tells the processor that the sensor is subject to Gaussian noise with:
```c
distanceWithNoise = libUncertainFloatGaussDist(distance, distance * sensorStdCoeff);
```
This creates a Gaussian distribution around the actual distance with a standard deviation that depends on the distance and a sensor-specific coefficient.

A key step of the algorithm is to update the distribution of the possible state with sensor readings.
Here, we offload the work to libUncertain to perform Bayesian inferences so this step becomes an one-liner (in [`main.c`](src/main.c)):
```c
estimatedPosition = libUncertainFloatBayesLaplace(&sensorModel, estimatedPosition, sensorReading);
```
where
* `sensorModel` is a function that models the sensor;
* `estimatedPosition` is the variable with an associated distribution containing the state of the filter;
* `sensorReading` is the value returned by the sensor.


## Options

You can play with this example by modifying the different parts:
* the distribution for the speed,
* the number and positions of landmarks,
* the sensor noise,
* and others.

Here are the options that you can set using the command line arguments:

```
Option summary
	[-h] [-k int] [-l float] [-a float] [-i float] [-e float] [-s float] [-c float]

Option details:

	-h: print this help message and exit

	-k int: maximum number of steps (default: 5)

	-l float: add a Landmark at the given position

	-a float: mAximal position (default: 10)

	-i float: mInimal position (default: -10)

	-e float: Exepcted speed (default: 1)

	-s float: speed Standard deviation (default: 1)

	-c float: sensor standard deviation Coefficient (default: 0.2)

	-p: output an SVG image instead of text
```

When no landmark is given, by default the code places a landmark in the middle of the allowed positions.
