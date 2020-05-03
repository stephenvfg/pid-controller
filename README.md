# Vehicle PID Controller

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

# Description

This repository contains the files for a project from the [Udacity Self Driving Car Nanodegree Program](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013). The goal of this project was to create a PID controller that would enable a vehicle to progress around a virtual racetrack based on specific parameters and error measurements.

<img src="https://github.com/stephenvfg/pid-controller/blob/master/vis.gif" width="500px">

The project code was compiled and used in tandem with a simulator from Udacity that placed the vehicle on the virtual racetrack. The simulator kept track of the cross-track error at each time interval while the vehicle moves so that we can use the cross-track error to inform the steering angle.

**Project code:**

* [Simulator interaction and steering calculation pipeline (`main.cpp`)](https://github.com/stephenvfg/pic-controller/blob/master/src/main.cpp)
* [PID controller logic (`PID.cpp`)](https://github.com/stephenvfg/pid-controller/blob/master/src/PID.cpp)
* [Twiddle coefficient optimization object (`Twiddle.cpp`)](https://github.com/stephenvfg/pid-controller/blob/master/src/Twiddle.cpp)

# PID Controllers

PID controllers are broken down into three distinct parameters each with their own coefficients and error values.

- `P` is the proportional parameter. Its effect is proportional to the distance between the vehicle's position and where the vehicle is meant to be on its path. We call this distance the "cross-track error" or CTE. If the `P` coefficient is too strong then the vehicle will swing wildly back-and-forth, if it's too weak then it will be too slow to react and get back on track.
- `I` is the integral parameter. Its effect is proportional to the total CTE over time and is used to react to systematic bias in the vehicle (steering drift for example). If the `I` coefficient is too strong then it risks over-reacting to small errors and steering the vehicle off course, if it's too weak then it will be ineffective at correcting any bias.
- `D` relates to the temporal derivative of the CTE. Its effect is proportional to the rate of change of the CTE or how quickly the vehicle adjusts its position with respect to its intended path. If the `D` coefficient is too strong then it will reduce how quickly the vehicle is able to course-correct, if it's too weak then the vehicle will over-correct and swing back-and-forth.

# Pipeline

The PID controller reads cross-track error data and uses it to produce a steering angle to help keep the vehicle on track.

1. Define the PID controller coefficients at line 41 of `main.cpp`. Use them to initialize the PID controller.
    - These coefficients were chosen through a combination of manual tuning and then Twiddle to further refine/optimize. See the Twiddle section below.
2. At each step, read the cross-track error for the vehicle. This is the delta between where the vehicle is and where the vehicle is meant to be based on its intended path.
3. Use the CTE to update the error for each PID parameter.
    - The P-error is simply the current CTE.
    - The I-error is the integral of the CTE since the first time interval (calculated as the sum of all CTE values).
    - The D-error is the derivative of the CTE since the last time interval (calulcated as the delta between the current CTE and last CTE).
4. Use the individual parameter errors to calculate the total error. The total error takes the sum of each PID parameter error multiplied by its respective coefficient (from step 1).
5. Set the steering value to be the value of the total error from the PID. Cap the steering value between [-1, 1].
6. The vehicle will move forward according to its speed, throttle, and steering value and then return to step 2 to repeat the process.

# Twiddle

The pipeline can optionally enable Twiddle to further optimize the given PID coefficients. When Twiddle is active, the simulator will repeatedly execute the same fixed number of iterations along the track over and over, slightly adjusting each PID coefficient at each restart and keeping track of the best error value for each cycle. Once Twiddle completes the optimization process, it holds onto the newly optimized PID coefficients. In the below example, we can see Twiddle complete and cycle and display the current coefficients and error value.

<img src="https://github.com/stephenvfg/pid-controller/blob/master/twiddle.gif" width="500px">

Twiddle can be activated in the pipeline by doing the following:

- Initialize Twiddle with a tolerance value at lines 49 and 50 of `main.cpp`. A tolerance value of `0.01` is a good place to start. Setting this to a smaller value have Twiddle run for longer and potentially return more precise values. 
- Set `bool do_twiddle = true` at line 52 of `main.cpp`.
- Set a value for `this->max_iteration` at line 30 of `twiddle.cpp`. This defines how long the vehicle will run for during each Twiddle loop before calculating error and resetting. A value of `1000` is a good place to start.
- Set values for the `this->dp` array of doubles. This determines how much Twiddle will initially add to or remove from the PID coefficients as it seeks out more optimal values. The smaller the values, the more refined the search will be (and the longer Twiddle might take to complete). The values `{Kp/10, Ki/10, Kd/10}` are a good place to start, setting the range to +/- 10% of the initial coefficient values.

To determine the optimal coefficients for my PID controller, I ran Twiddle through several cycles, increasing the number of iterations and decreasing the tolerance each time to further refine the accuracy of the coefficients. Below are some charts that illustrate how Twiddle works.

| `P` coefficient  | `I` coefficient | `D` coefficient |
| ---------------- | --------------- | --------------- |
| <img src="https://github.com/stephenvfg/pid-controller/blob/master/data/p.png" width="300px"> | <img src="https://github.com/stephenvfg/pid-controller/blob/master/data/i.png" width="300px"> | <img src="https://github.com/stephenvfg/pid-controller/blob/master/data/d.png" width="300px"> |

Since Twiddle runs until the delta values shrink below the defined threshold, it's important to also track that sum. This chart illustratoes how the sum decreases as the coefficients become more and more optimal. The sum increases each time we find a better coefficient and decrease each time otpimal coefficient set wins over the test set.

| Sum of the `dp` values at each cycle |
| ------------------------------------ |
| <img src="https://github.com/stephenvfg/pid-controller/blob/master/data/sum.png" width="300px"> |