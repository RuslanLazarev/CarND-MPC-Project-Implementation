# Udacity Self-Driving Car Engineer Nanodegree Program
  
## *Model Predictive Control of Autonomous Vehicle*
  
---
  
  
### The Model
  
The model has been thoroughly described in the classroom.  The state and control vector are given by:
<img src="https://latex.codecogs.com/gif.latex?&#x5C;vec{x}_t%20=%20&#x5C;left(%20&#x5C;begin{array}{c}	x_t&#x5C;&#x5C;%20%20%20%20y_t&#x5C;&#x5C;%20%20%20%20&#x5C;psi_t&#x5C;&#x5C;%20%20%20%20cte_t&#x5C;&#x5C;%20%20%20%20&#x5C;Delta&#x5C;psi_t	&#x5C;end{array}&#x5C;right),%20&#x5C;vec{u}_t%20=%20&#x5C;left(%20&#x5C;begin{array}{c}	&#x5C;delta_t&#x5C;&#x5C;%20%20%20%20a_t	&#x5C;end{array}&#x5C;right)"/>
  
where <img src="https://latex.codecogs.com/gif.latex?x"/>, <img src="https://latex.codecogs.com/gif.latex?y"/> and <img src="https://latex.codecogs.com/gif.latex?&#x5C;psi"/> are the vehicle position and heading in the vehicle coordinate system. Other states are the cross track error with respect to our fitted polynomial, <img src="https://latex.codecogs.com/gif.latex?cte"/>, and the deviation between the vehicle heading and the desired heading, <img src="https://latex.codecogs.com/gif.latex?&#x5C;Delta&#x5C;psi"/>. The control vector is given by the steering angle and the vehicle's acceleration.
  
<img src="https://latex.codecogs.com/gif.latex?x_{t+1}%20=%20x_t%20+%20v_t&#x5C;cos(&#x5C;psi_t)&#x5C;Delta%20t"/>
<img src="https://latex.codecogs.com/gif.latex?y_{t+1}%20=%20y_t%20+%20v_t&#x5C;sin(&#x5C;psi_t)&#x5C;Delta%20t"/>
<img src="https://latex.codecogs.com/gif.latex?&#x5C;psi_{t+1}%20=%20&#x5C;psi_t%20-%20&#x5C;frac{v_t}{L_f}%20&#x5C;delta_t%20&#x5C;Delta%20t"/>
(Equation for <img src="https://latex.codecogs.com/gif.latex?&#x5C;psi_{t+1}"/> has been changed from summation due to in the simulator a positive value implies a right turn and a negative value implies a left turn.)
<img src="https://latex.codecogs.com/gif.latex?v_{t+1}%20=%20v_t%20+%20a_t%20&#x5C;Delta%20t"/>
<img src="https://latex.codecogs.com/gif.latex?cte_{t+1}%20=%20f(x_t)%20-%20y_t%20+%20v_t&#x5C;sin(&#x5C;Delta&#x5C;psi_t)&#x5C;Delta%20t"/>
<img src="https://latex.codecogs.com/gif.latex?&#x5C;Delta&#x5C;psi_{t+1}%20=%20&#x5C;psi_t%20-%20&#x5C;psi_{desired,t}%20-%20&#x5C;frac{v_t}{L_f}%20&#x5C;delta_t%20&#x5C;Delta%20t"/>
  
The constraints are: <img src="https://latex.codecogs.com/gif.latex?&#x5C;delta%20&#x5C;in%20[-25^o,%2025^o]"/> and <img src="https://latex.codecogs.com/gif.latex?a%20&#x5C;in%20[-1,1]"/>.
The model is implemented in `MPC.cpp`. `Ipopt` library was used to model contraints. MPC solver is employed to minimize the  objective:
  
<img src="https://latex.codecogs.com/gif.latex?J%20=%20&#x5C;sum^N_{t=1}%20&#x5C;lambda_1&#x5C;cdot(cte^2)%20+%20&#x5C;lambda_2&#x5C;cdot(&#x5C;Delta&#x5C;psi_t)^2%20+%20&#x5C;lambda_3&#x5C;cdot(v_t^2&#x5C;cdot&#x5C;alpha)^2%20%20+%20&#x5C;dots"/>
  
The thrid term was taken into consideration to slow down the vehicle when road has curves (high slopes).
  
CppAD library was used for automatic differentiation.
  
### Timestep Length and Elapsed Duration
  
the values was chosen as <img src="https://latex.codecogs.com/gif.latex?N=10"/> and <img src="https://latex.codecogs.com/gif.latex?dt=0.1"/>. These values were discussed in offfice hours session. Other values were tested (starting from default from classroom <img src="https://latex.codecogs.com/gif.latex?N=25"/> and <img src="https://latex.codecogs.com/gif.latex?dt=0.05"/>) yet a trade-off should be reached for an optimal resolution (<img src="https://latex.codecogs.com/gif.latex?dt"/>) and computattional time (<img src="https://latex.codecogs.com/gif.latex?N"/>) for real-time tasks. While in the prevoius submission the cars drove much faster (there was no kinematic model computation to deal with latency), the updated controller takes longer time on high road curves. Elapsed duration and timesteps lengths together affect the predicted path and the vehicle speed. Long time horizon is harder to calculate on sharp curves, which slows down the car. Algorithm will struggle with polynomial fitting and this will add chaotic behavior to the car's driving.
  
  
### Polynomial Fitting and MPC Preprocessing
  
The waypoint were processed in `main.cpp` file to transform the into vehicle perspective (taking the technique from office hours session). Afterwards, polynomial fitting is easier because orientation angle is 0 and vehicle coordinates are at the origin. Polynomial fitting was done as in Lesson 18.
  
  
### Model Predictive Control with Latency
  
A contributing factor to latency is actuator dynamics. Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency. 
  
A kinematic model has been used to deal with latency (as proposed after the first submission review). Here, the controller has used kinematic equationsto predict the states after <img src="https://latex.codecogs.com/gif.latex?100&#x5C;mu%20s"/> before sending them to MPC solver. The update is placed after polynomial fitting and uses vehicle map coordinates.
  
  
  
---
## Dependencies
  
  
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/ )
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/ )
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm )
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/ )
  * Windows: recommend using [MinGW](http://www.mingw.org/ )
* [uWebSockets](https://github.com/uWebSockets/uWebSockets )
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3 ) for more details.
  
* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md ) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page ). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases ).
* Not a dependency but read the [DATA.md](./DATA.md ) for a description of the data sent back from the simulator.
  
  
## Basic Build Instructions
  
  
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
  