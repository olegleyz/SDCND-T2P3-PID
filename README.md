# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## PID Therory
### Proportional Response (P)
The proportional component depends only on the difference between the set point and the process variable. This difference is referred to as the Error term. The proportional gain (Kp) determines the ratio of output response to the error signal. For instance, if the error term has a magnitude of 10, a proportional gain of 5 would produce a proportional response of 50. In general, increasing the proportional gain will increase the speed of the control system response. However, if the proportional gain is too large, the process variable will begin to oscillate. If Kp is increased further, the oscillations will become larger and the system will become unstable and may even oscillate out of control.

### Integral Response (I)
The integral component sums the error term over time. The result is that even a small error term will cause the integral component to increase slowly. The integral response will continually increase over time unless the error is zero, so the effect is to drive the Steady-State error to zero. Steady-State error is the final difference between the process variable and set point. A phenomenon called integral windup results when integral action saturates a controller without the controller driving the error signal toward zero.

### Derivative Response (D)
The derivative component causes the output to decrease if the process variable is increasing rapidly. The derivative response is proportional to the rate of change of the process variable. Increasing the derivative time (Td) parameter will cause the control system to react more strongly to changes in the error term and will increase the speed of the overall control system response. Most practical control systems use very small derivative time (Td), because the Derivative Response is highly sensitive to noise in the process variable signal. If the sensor feedback signal is noisy or if the control loop rate is too slow, the derivative response can make the control system unstable. [1]

## Reflection

In this project the car is driving in the simulator. The throttle is a fixed value, steering angle is managed by the PID controller.
I've selected the initial coefficients manually. I've used the approach described at https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops for this:
*  Set all gains to zero.
*  Increase the P gain until the response to a disturbance is steady oscillation.
*  Increase the D gain until the the oscillations go away (i.e. it's critically damped).
*  Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
*  Set P and D to the last stable values.
*  Increase the I gain until it brings you to the setpoint with the number of oscillations desired (normally zero but a quicker response can be had if you don't mind a couple oscillations of overshoot).
With this approach I've chosen coefficients (0.2, 0.001, 16) for Kp, Ki, Kd respectively.

Then I've implemented twiddling technique, proposed by Sebastian Thrun in his lecture, for fine tuning of the coefficients.
Twiddle is prone to find a local minima - that's why I was not able to find optimal starting point using twiddling.

The approach was for twiddling was to drive during 1000 time steps, starting to track the error after first 100 steps.

The coefficients after twiddling are: Kp: 0.225, Ki: 0.001, Kd: 14.8421.

## Literature
1.  PID theory explained. http://www.ni.com/white-paper/3782/en/