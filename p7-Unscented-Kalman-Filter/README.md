# Unscented Kalman Filter
Self-Driving Car Engineer Nanodegree Program

---

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

INPUT: values provided by the simulator to the c++ program

	["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

	["estimate_x"] <= kalman filter estimated position x
	["estimate_y"] <= kalman filter estimated position y
	["rmse_x"]
	["rmse_y"]
	["rmse_vx"]
	["rmse_vy"]


## Result Demonstration
#### Video demonstration dataset1 and dataset2

[![](https://img.youtube.com/vi/F7sMPR5R4yI/0.jpg)](https://youtu.be/F7sMPR5R4yI)


---


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.


* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF`


## UKF Algorithm Desciption
- Receive sensor data
- Determine sensor type
- Process sensor data
	- Initlize if it is the first measurement
	- Create augmented sigma points
	- Predict the sigma points
	- Predict mean and covariance of sigma points
	- Update the state and its state covariance matrix


## Result Comparison
To demonstrate the performance of Unscented Kalman Filter, compare its performance using both lidar and radar with using only one of them and Extended Kalman Filter.

The RMSE result is as followed:

	RMSE: [x, y, vx, vy]

	UKF (both sensors)
	[0.0613, 0.0838, 0.3189, 0.2348]

	UKF (LIDAR)
	[0.0981, 0.0966, 0.4707, 0.2338]

	UKF (RADAR)
	[0.1474, 0.1952, 0.3543, 0.2327]

	EKF
	[0.1325, 0.1952, 0.6923, 0.8470]