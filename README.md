# PID-Control-Project

This is my 4th project of Udacity SDCNanodegree program term 2, which aims to utilize a PID controller and to tune the hyperparameters in C++ to maneuver the vehicle around the lake race track. The simulator will provide the crosstrack error(CTE)and speed(mph) to compute the appropriate steering angle. The PID controller must drive the vehicle smoothly with steering and throttle commands around the simulator track.

---
[//]: # (Image References)

[image1]: ./PID_twiddle.png "PId Visualization"
[image2]: ./TuningTable.png


## Hyperparameter Tuning/Optimization


There are two ways to tune the PID control gains(proportional factor Kp, intergral factor Ki, and derivative factor Kd), refering to [George Gillard's introction and tutorial for PID controllers](https://udacity-reviews-uploads.s3.amazonaws.com/_attachments/41330/1493863065/pid_control_document.pdf).

### A. Manual tuning 

One is to tune them manually, modify these factors and run them on simulator each time to see the CTE and steer_value outputs and vehicle performance.

#### 1.Describe the effect each of the P, I, D components had in your implementation. Is it what you expected? Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.

Generally speaking, in PID system:

- **the P(Proportional)-term** is proportional to the distance between the vehicle and the reference trajectory, i.e.the CrossTrack Error(CTE). And it can lead to steering overshoot easily. In this project, this P-term has the most obvious effect on vehicle's behavior. It can cause the vehicle to steer opposite proportional to the vehicle's distance from the reference lane center. If the vehicle has a huge bias to right it will steer hard to the left, and if the vehicle has a slight bias to left it will steer slightly to the right.

- **the D(Derivative)-term** is used to avoid oscillations for smooth turning and driving. In this project, D-term counteracts the P-term's tendency to osillate and overshoot the lane center. A proper Kd will lead the vehicle to approach the lane center smoothly without oscillations.

- **the I(Integral)-term** is the intergral sum of all previous deviations from reference path and is used to remove the systematic bias, like steering drift or bad vehicle aligment. In this project, I-term can push CTE to 0, which means to help PD controller reach the exact lane center. 


The effects of these three components for this controller are listed in the following figure, respectively:

![alt text][image1]


#### 2. Describe how the final hyperparameters were chosen.
According to the [udacity forum](https://discussions.udacity.com/t/how-to-tune-parameters/303845/4) and [George Gillard's document](https://udacity-reviews-uploads.s3.amazonaws.com/_attachments/41330/1493863065/pid_control_document.pdf),The process of tuning coefficients is as follows:

- 1.set Kp,Ki and Kd to 0. This will disable them for now.
- 2.Increase Kp until the error is fairly small, but it still gets from the beginning to nearly the end quickly enough.
- 3.Increase Kd until any overshoot you may have is fairly minimal. But be careful with Kd – too much will make it overshoot.
- 4.Increase Ki until any error that is still existing is eliminated. Start with a really small number for Ki, don't be surprised if it is as small as 0.0001 or even smaller.
- 5.Using the rules of tuning the constants as in the following figure, you can change around the constants a little bit to get it working to the best performance.
![alt text][image2]

Base on many trials and errors and the experience communications from forum, I found that too high Kp and too low Kp will produce fast-decreasing oscillation or slow-decreasing oscillation, respectively. And too high Kd and too low Kd will lead to overdamp and underdamp, respectively. The Ki should be very small. Only the proper PID gains can drive the vehicle smoothly. And each representative tuning step is recorded as a .mp4 video using kazam under Ubuntu 14.04LTS.

So I set the three parameters: Kp ~= 0.1Kd and Kp ~= 0.01Kp. The initial Kp is 1.0.

trial#|Gains(Kp,Ki,Kd)|  Video | Performance Description|
------|-----------------|-----------|-------------
trial1|(1.0, 0.0, 0.0)  |./demos/trial1.mp4 | huge oscillation and vehicle drives horriblely out of track
trial2|(0.1, 0.0, 0.0)  |./demos/trial2.mp4 | vehicle drives more smoothly but still is out of track during the first turn
trial3|(0.13,0.0, 0.0)  |./demos/trial3.mp4 | vehicle drives very smoothly in starting straight lane then out of track during the first turn,with much lower cte 
trial4|(0.13,0.0, 1.0)  |./demos/trial4.mp4| add Kd = 1.0 and vehicle finishes the whole track very smoothly
trial5|(0.13,0.0, 3.0)  |./demos/trial5.mp4| increase Ki and vehicle finishes the whole track while with more bumps and oscillations during the 2nd and 3rd turn.
trial6|(0.13,0.0001,1.0)|**FinalChoice**|add Ki and vehicle finishes the whole track very smoothly with almost cte=0.  
trial7|(0.13,0.0002,3.0)|./demos/trial7.mp4| increase Ki and vehicle finishes the whole track but larger bumps during turns.

**Therefore, my final choice about PID gains(Kp,Ki,Kd) = trial 6(0.13, 0.0001,1.0). Please refer to ./demos to see the FinalChoice.mp4.** Besides, I have recorded trail1-trail7.mp4 perfectly to prove my tuning process, but the videos are too large(~2GB)and I have tried half a week to upload them online because of github attachment constraints. Now please see them.

### B. Efficient tuning- Twiddle

The other is to apply efficient tuning algorithms automatically, like Twiddle, Ziegler-Nichols method, SGD or a combination.

For now I haven't implemented the function Twiddle()in this project. And I will update this section later.
	

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

---
## How to run this project

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

