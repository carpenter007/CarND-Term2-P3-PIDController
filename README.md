# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Project Description
In this project, a PID controller implementation is used to control the steering value of a car, driving around the Udacity simulator track. Additionally a second PID controller is used to control the speed.

### Tuning parameters

Tuning the parameters is done manually, but based on the theory which is behind the Ziegler-Nichols tuning model. Using Twiggle for determining the parameter gain was not practicable for me. It did not match the trade-off between smooth driving, speed and safety.

The following table shows what effects the different componentes P, I, D of the PID controller.



| Response | Rise Time | Overshoot | Settling Time | S-S Error* |
| :------: | :-------: | :-------: | :-----------: | :--------: |
| K_P      | Decrease  | Increase  | NT            | Decrease   | 
| K_I      | Decrease  | Increase  | Increase      | Eliminate  |
| K_D      | NT        | Decrease  | Decrease      | NT         |

\* Steady-state Error: the difference between the steady-state output and the desired output.

### First tuning process

In the first tuning process I tuned a PID controller for the steering value of the car. First I determined a good value for the proportional component P. Only using the P component results in a cyclic overshooting. I managed the P value to be high enough for the first curves, but not overfitting that much, that the car gets of the road.
Second, to reduce the overshoots, I tuned the differentially proportionally D parameter. This parameter also helps to increase the steering value when the CTE increases strongly (e.g. when a curve occures after driving straight forward)
When I recoginze that the car takes a long distance, until it is in stable in the middle of the street, then I increased the integral parameter I value.
 0.15, 0.002, 1.8 was a good value set a constant throttle value of 0.3 (~30mph is possible) The high D value helps to avoid crossing the lane. The strongly increased parameter D has the effect that some steerances look not very smooth.

### Second tuning process

In order to to increase the speed and get a smoother behaviour, I introduces a further PID controller for throttle displacement. The main idea is, to reduce the speed of the car whenever I get a high change of CTE. Doing so, I can reduce the parameter D on the steering PID controller. Decreasing the D value forces me also to decrease the other both values to avoid the continues overshoots.

### Third tuning process

Finding a compromise between safe driving, speed and also smoothness, I started a third tuning process. In this tuning processes, still two PID controllers are used (for steering and throttle).
Main idea in this process is, to setup a very sensitive throttle control, which brakes the car, depending on the CTE.
The steering is calculated by a PID controller as before. But additionally I reduce / increase the PID parameter set depending on the current speed. This allowes the car to drive smoother on higher speeds. If the car brakes (because of a high CTE), the PID parameters for the steering are higher, which allows to drive safely in dangerous situations.

### Final parameter values
The following two lines describe the final parameters for steering and throttle PID controller:

```c++ 
pid_steering.Init(0.05, 0.000015, 0.6);
pid_speed.Init(0.06, 0.0001, 12);
```

The resulting steering value will be multiplied by the factor (100 / speed), whereas the typical speed value is between 20 and 70 mph.


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


## Code Style

Code Style sticks to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) *.

 \* set tab width to 2 spaces (keeps the matrices in source code aligned)



