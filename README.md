# Writeup for P4 of SDC Nanodegree Term2 : PID Control
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

* I (Integral) : This term sums up all CTEs, so that it prevent the car from moving along one side of the lane.

* D (Derivative) : This term sense how fast CTE change. Therefore the controller gives large steering angle when CTE changes rapidly, but on the contrary the controller gives small steering angle when CTE changes slowly.

### 2. Parameter Optimization

I used **Twiddle** algorithm for the optimization of parameters of PID controller. The explanation of the algorithm is provided at [the lecture from Udacity](https://www.youtube.com/watch?v=2uQ2BSzDvXs). The lecture gave me the code of twiddle in python like the following. I built another version of twiddle code in C++ in this project. It can be found in `PID.cpp`.  

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

In addition to PID contoller for steering angle, I made the throttle value changes in proportional to CTE. The function of throttle value is shown in the below.

#### throttle = -0.08*(CTE ^ 2) + 0.4

![alt_text][image2]

Therefore I could reduce maximum CTE compared to the case of constant throttle value. It was confirm by the change of accumulated error during a single lap. (0.394838 -> 0.369452)
