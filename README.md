# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Implementation of Model

A Kinematic model was used for the controller. Tire force and gravity are ignored in this model.The model equations are as follow:

          x_next = x_i + v * cos(psi_i) * dt;
          
          y_next = y_i + v * sin(psi_i) * dt;
          
          psi_next = psi_i - v * steering_angle * dt / mpc.Lf;
          
          v_next = v + throttle * dt;
          
          cte_next = cte_i + v * sin(epsi_i) * dt;
          
          epsi_next = epsi_i - v * steering_angle * dt / mpc.Lf; 
          

Where:

dt is the time interval

x, y is the x and y coordinates of Car's position.

psi is car's heading direction.

v is velocity.

cte is cross-track error.

epsi is orientation error.

Lf is the distance between the center of mass and the front wheels. 

## MPC 

The number of watpoints(N) and the time interval(dt) define the prediction time horizon. The time horizon T was chosen to 1s. The number of points impacts the controller performance as well. Time step interval dt was set to the latancy of the simulation 100 ms. Number of waypoints N is 10.

The waypoints data from the simulator are transformed to the vehicle coordinate system. Then a 3rd-degree polynomial is used to fit the waypoints. Cross-track error(cte) and orientation error(epsi) are calculated through the polynomial fitting. Then they are used by the MPC solver to calculate steer value and throttle value.

In the Model Predictive Control that handles a 100 millisecond latency, the state values are calculated with given time interval 0.1s.  These state values are updated from the initial values. This part of code is implemented in main.cpp at line 130.

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

To run the code:

Set the build directory as current directory, then run ./pid.

The following words will be shown on screen: Listening to port 4567.

Then run the simulator. In the main menu screen select Project 5: MPC Controller
