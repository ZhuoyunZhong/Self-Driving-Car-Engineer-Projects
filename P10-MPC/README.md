# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Result Demonstration
### Video demonstration

[![](https://img.youtube.com/vi/8qw0ykeOhuI/0.jpg)](https://youtu.be/8qw0ykeOhuI)


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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

**Only tested in Linux system**

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Model Predictive Control Description
**Model Predictive Control (MPC)** is an advanced method of process control that is used to control a process while satisfying a set of constraints. MPC has the ability to anticipate future events and can take control actions accordingly.

The MPC has four main parts to define:

- **Define the duration of the trajectory **T****. This hypermeter T is the result of N * dt. dt is the time elapses between actuations and N is the number of timesteps. It will influence the accuracy of approximated trajectory.

- **Define the vehicle model.** The vehicle model used is bycicle model descriped before. The state is therfore defined as: **`[x, y, psi, v, cte, epsi]`** where x, y are the positions of the vechicle, psi is the heading direction, v is the velocity, cte is the cross-track error and epsi is the orientation error.

- **Define constraints.** In reality, actuators have limits. For example, a usual vehicle can never steer 60° to turn. Therefore, some limits of the actuators value are set: **`Steering angle: [-25°,25°], acceleration: [-1, 1]`**

- **Define cost function.** Cost function is the function based on whom the MPC solver trys to optimize. The cost function consists of many different terms with different weights. In this model, not only the **cross-track error** and **heading error** are included, but **the use of actuators** and **the change of sequential actuations** are considered as well.


The **solving process** is as below:

**`Vehicle current State -> MPC Solver -> Vehicle control Input`**

Though with all the control input results given, only the first control input will be implemented. Then repeat the loop.



## Implementation Detail 

### Choice of hyperparameters
**N** is the number of timesteps in the horizon. **dt** is how much time elapses between actuations. **T** is the duration over which future predictions are made. 

The relation of them is: **`T = N * dt`**

In the case of driving in the simulator, a choice of T between **0.4** seconds and **0.6** seconds is reasonable. A smaller choice will lead to an inaccurate prediction and a larger choice won't make sense due to the change of environment in the near future.

For N and dt, a too large choice of N will demand optimizer to spend much computational resource to calculate delta and a. What is more, a large dt will make it harder to approximate the ideal trajectory.

Above all, I tested **T = 0.4** and **T = 0.6**. In each condition I tried **dt = 0.05** and **dt = 0.1**. I observed that **dt = 0.1** will sometimes returned unstable trajectory, thus not a good approximation. Therefore a **dt = 0.05** is chosen for both situation. Also, I found that **T = 0.6** will have a better performance especially in the cases of sudden turn. 

Accordingly, I chose **T = 0.6**, **dt = 0.05**, **N = 12**


### Waypoints prepocessing
Waypoints returned from the simulator are based on global coordinate while the visualization of trajectories should be on local coordinate. Therefore, a **coordinate transform** is implemented before fitting curves.

	for (size_t i = 0; i < ptsx.size(); ++i){
		// shift
		double local_x = ptsx[i] - px;
		double local_y = ptsy[i] - py;
		// rotate
        ptsx[i] = cos(-psi) * local_x - sin(-psi) * local_y;
        ptsy[i] = sin(-psi) * local_x + cos(-psi) * local_y;
    }

After this implemention and curve fitting, the state of the vehicle changes into 

`x = 0` `y = 0` `psi = 0` 

and accordingly 

`cte = polyeval(coeffs, 0) - 0`
`epsi = - atan(coeffs[1])`


### Latency problem
In reality, control method always has a latency problem. In this project, the program delays 100 ms to send control statement in order to simulate the latency process.

To deal with this problem, the control statement should not base on the state of current time any more but on the state of 100 ms in the future.

I use the same model to predict the vehicle state after 100 ms.

	double future_px = 0.0 + v * latency;
	double future_py = 0.0;
	double future_psi = 0.0 + v * -delta / Lf * latency;
	double future_v = v + a * latency;
	double future_cte = cte + v * sin(epsi) * latency;
	double future_epsi = epsi + v * (-delta) / Lf * latency;

And the state I use to pass to MPC and solve is now:

`state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;`