# Code Organization

We have split this implementation into multiple files to make it easier to understand and modify.

* `environment.[hc]` has information about the environment: the environment's bounds and the landmarks' position.
* `model.[hc]` contains a model for the motion and the sensor.
* `system.[hc]` simulates the motion and the sensor.
* `plot.[hc]` produces an SVG plot.
* `main.c` is the particle filter.
