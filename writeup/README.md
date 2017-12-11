# **Model Predictive Control (MPC) Project**

### Vehicle control using the MPC model

---

**Model Predictive Control (MPC) Project**

The goals / steps of this project are the following:

* Complete starter code to succesfully drive vehicle in simulator with MPC model


---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

My project includes the following files:
* [main.cpp](../src/main.cpp) main program that runs the IO server (given from starter code), fits the polynomial trajectory, and handles the actuator delays
* [MPC.cpp](../src/MPC.cpp) implements the Model Predictive Control class
* [MPC.h](../src/MPC.h) header file for [MPC.cpp](../src/MPC.cpp)
* [TrackRecording.mp4](./TrackRecording.mp4) video of a single lap around the course in the simulator

### Discussion

#### 1. Implementation

The implementation of the MPC controller was done in both [MPC.cpp](../src/MPC.cpp) and [main.cpp](../src/main.cpp).

Modifications to [MPC.cpp](../src/MPC.cpp) were the following:

* [Timestep and length assignment](../src/MPC.cpp#L12)
* [Setpoint assignments](../src/MPC.cpp#L16)
* [Start position assignments](../src/MPC.cpp#L19)
* [Cost weight assignments](../src/MPC.cpp#L29)
* [FG_eval](../src/MPC.cpp#L38)
 * [Cost function calculation](../src/MPC.cpp#L46)
  * [Costs based on error relative to reference state](../src/MPC.cpp#L46)
  * [Costs to minimize use of actuators](../src/MPC.cpp#L56)
  * [Costs to minimize actuator discontinuities](../src/MPC.cpp#L62)
 * [Constraint assignments](../src/MPC.cpp#L68)
  * [Time 0 constraints](../src/MPC.cpp#L76)
  * [Time 1 constraints](../src/MPC.cpp#L91)
  * [Other constraints](../src/MPC.cpp#L99)
* [MPC::Solve](../src/MPC.cpp#L117)
 * [Assignment of model variables](../src/MPC.cpp#L122)
 * [Initialization of vars](../src/MPC.cpp#L127)
 * [Assignment of vars bounds](../src/MPC.cpp#L142)
 * [Assignment of constraints bounds](../src/MPC.cpp#L164)
 * [Return from CppAD::ipopt solver](../src/MPC.cpp#L239)

And modifications to [main.cpp](../src/main.cpp) were the following:

* [Translation of waypoints to car's perspective](../src/main.cpp#L99)
* [Get 3rd degree polynomial coefficients that describe the trajectory](../src/main.cpp#L109)
* [Create state matrix from car's perspective with delay](../src/main.cpp#L112)
* [Solve with MPC](../src/main.cpp#L126)
* [Extract actuator values from solution](../src/main.cpp#L129)
* [Create points to draw MPC's solution path](../src/main.cpp#L143)
* [Create points to show desired trajectory](../src/main.cpp#L155)

For the most part example code from the coursework was used.  Special considerations were transforming of the trajectory to the car's perspective and also the addition of a delay in the state matrix.  The state matrix's values consider the delayed actuation.  Before this implementation there was a tendency to oscillate in the control due to the controller always attempting to catch up.

Tuning of the model involved a lot of back and forth between the cost weights and the timestep/duration.  Initially it was difficult to distinguish which needed adjustment, but as the model became better tuned it was easier to distinguish the cause and effect of each.  The model was initially first without any time delay and a lower reference velocity.  To reduce the tendency to oscillate, the weights for actuator discontinuities was increased to prevent sudden movements that would initiate an oscillation, but after the delay implementation in the model these and the reference velocity could be increased as well. 

#### 2. Results

The final results were very desirable. The added consideration of delay in the model dampens oscillation and the vehicle is able to travel around the track at high speed without any oscillation at all.  Additionally the vehicle stays well centered and the MPC's solution can be clearly observed trying to pull the vehicle back on path.  Specifically noteworthy is the difference in paths around turns, where the MPC solution can be seen undershooting the path, where some cross-track error is intentional, but then bringing the vehicle back on path.  This is due to other costs outweighing the cross-track error in the turns.

Results:

[TrackRecording.mp4](./TrackRecording.mp4)


