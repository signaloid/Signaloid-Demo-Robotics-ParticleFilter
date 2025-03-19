# Source code:

## main.c
Implementation of the particle filter.

## environment.c/h
Has information about the environment: the environment's bounds and the landmarks' position.

## model.c/h
Contains a model for the motion and the sensor.

## system.c/h
Simulates the motion and the sensor.

## plot.c/h
Produces an svg plot.

## datatypes.h
Contains shared datatypes.

## utilities.c/h
These contain utility methods for parsing, setting, and reporting
the usage of demo-specific command-line arguments of C/C++ demo applications.
These methods call similar methods from `common.c` for handling
command-line arguments common to all of our C/C++ demo applications.

## common.c/h
These contain utility methods for parsing, setting, and reporting
the usage of command-line arguments common to all of our C/C++ demo applications,
as well as other methods that we commonly make use across our
C/C++ demo applications, e.g., standard methods for I/O handling. These
source files are symlinks to the original files contained in the repository
[Signaloid-Demo-CommonUtilityRoutines](https://github.com/signaloid/Signaloid-Demo-CommonUtilityRoutines)
which is included as a submodule in `submodules/common`.

## config.mk
Signaloid cores use this file to identify the source codes they will use when
building the C/C++ demo application.
