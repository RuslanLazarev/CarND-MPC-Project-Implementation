# Udacity Self-Driving Car Engineer Nanodegree Program
## *Model Predictive Control of Autonomous Vehicle*
---


### The Model
The model has been thoroughly described in the classroom.  The state and control vector are given by:
$\vec{x}_t = \left( \begin{array}{c}
	x_t\\
    y_t\\
    \psi_t\\
    cte_t\\
    \Delta\psi_t
	\end{array}\right), \vec{u}_t = \left( \begin{array}{c}
	\delta_t\\
    a_t
	\end{array}\right)$
where $x$, $y$ and $\psi$ are the vehicle position and heading in the vehicle coordinate system. Other states are the cross track error with respect to our fitted polynomial, $cte$, and the deviation between the vehicle heading and the desired heading, $\Delta\psi$. The control vector is given by the steering angle and the vehicle's acceleration.
$x_{t+1} = x_t + v_t\cos(\psi_t)\Delta t$
$y_{t+1} = y_t + v_t\sin(\psi_t)\Delta t$
$\psi_{t+1} = \psi_t - \frac{v_t}{L_f} \delta_t \Delta t$
$v_{t+1} = v_t + a_t \Delta t$
$cte_{t+1} = f(x_t) - y_t + v_t\sin(\Delta\psi_t)\Delta t$
$\Delta\psi_{t+1} = \psi_t - \psi_{desired,t} - \frac{v_t}{L_f} \delta_t \Delta t$
The model is implemented in MPC.cpp. Ipopt library was used to model contraints. MPC solver is employed to minimize the  objective:

$J = \sum^N_{t=1} \lambda_1\cdot(cte^2) + \lambda_2\cdot(\Delta\psi_t)^2 + \lambda_3\cdot(v_t^2\cdot\kappa)^2 + \dots$


### Timestep Length and Elapsed Duration



### Polynomial Fitting and MPC Preprocessing



### Model Predictive Control with Latency




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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
