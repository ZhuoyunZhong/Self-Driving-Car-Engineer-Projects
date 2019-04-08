# Kidnapped Vehicle
Self-Driving Car Engineer Nanodegree Program

---

## Project Introduction
The robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project a 2 dimensional particle filter is implemented in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Result Demonstration
#### Video demonstration

[![](./demonstration/p8.gif)](https://youtu.be/fUX9pNs2IIY)


## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Particle Filter Directory
The directory structure of this repository is as follows:

	root
	|   build.sh
	|   clean.sh
	|   CMakeLists.txt
	|   README.md
	|   run.sh
	|
	|___data
	|   |   
	|   |   map_data.txt
	|   
	|   
	|___src
	    |   helper_functions.h
	    |   main.cpp
	    |   map.h
	    |   particle_filter.cpp
	    |   particle_filter.h

## Particle Filter Implementation
### Algorithm Desciption
- Set particle filters parameters
- Initialize location with GPS
- Predict the next location based on vehicle control data
- Update all the particles' weight
	- Transform observation coordinate
	- Associate observations with landmarks
	- Calculate each particle weight based on Gaussian distribution
- Resample particles according to particles' weight
- Repeat the process from prediction to resampling.


### Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

##### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

##### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.