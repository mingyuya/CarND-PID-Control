# Writeup for P4 of Self-Driving Car Nanodegree Term2: PID Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Matt, MinGyu, Kim
---

[image1]: https://upload.wikimedia.org/wikipedia/commons/thumb/4/43/PID_en.svg/400px-PID_en.svg.png "PID"
[image2]: ./figures/graph_throttle.png "GRAPH_THROTTLE"

## Introduction

[PID Controller](https://en.wikipedia.org/wiki/PID_controller) is widely used in industrial control systems. It contains a control loop feedback to calculate an error value as the difference between a desired setpoint and measured variable and it is propagated to propotional, integral and derivative terms to calculate a correction value.
The following is a block diagram of a PID controller

![alt_text][image1]

More theoretical details can be found at the link above.

## Implementation

### 1. PID Controller
In this project, the goal is steering the car in the simulator safely by PID controller. To make it possible, I have to implement the PID controller calculating steering angle of the car from a CTE (Cross-Track Error). Here are more about each terms of PID.

* P (Propotional) : This term make the car steer in propotional to the CTE. Therefore the car keeps heading to the center when everytime it goes far from the center of a lane. However, if the parameter of this is setted too high, the car will repeat approaching to the center and receding from the center very frequently. (i.e., the car is oscillate). On the other hand, for the low value of the parameter, the car will not follow the track when it meets a rapid curve.

* I (Integral) : This term sums up all CTEs until 

* D (Differential) :

### 2. Parameter Optimization

I used **Twiddle** algorithm for the optimization of parameters of PID controller. The explanation of the algorithm is provided at [here](https://www.youtube.com/watch?v=2uQ2BSzDvXs). In addition, the following is

```{.python}
# Choose an initialization parameter vector
p = [0, 0, 0]
# Define potential changes
dp = [1, 1, 1]
# Calculate the error
best_err = A(p)

threshold = 0.001

while sum(dp) > threshold:
    for i in range(len(p)):
        p[i] += dp[i]
        err = A(p)

        if err < best_err:  # There was some improvement
            best_err = err
            dp[i] *= 1.1
        else:  # There was no improvement
            p[i] -= 2*dp[i]  # Go into the other direction
            err = A(p)

            if err < best_err:  # There was an improvement
                best_err = err
                dp[i] *= 1.05
            else  # There was no improvement
                p[i] += dp[i]
                # As there was no improvement, the step size in either
                # direction, the step size might simply be too big.
                dp[i] *= 0.95
```

### 3. Adaptive throttle value

![alt_text][image2]

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.



## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

