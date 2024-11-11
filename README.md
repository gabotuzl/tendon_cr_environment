# tendon_cr_environment
ROS2 environment for the simulation and control of a tendon-driven continuum robot. 

The purpose of this project is to enable users to run simulations and implement control schemes for a tendon-driven continuum robot. ROS2 Humble was utilized to construct the interactive environment, which allows for the integration of multiple functions operating in parallel and facilitates the transition to the physical implementation of the robot. The simulation of the continuum robot is performed using PyElastica (https://github.com/GazzolaLab/PyElastica). This project’s open-source nature is intended to facilitate code inspection and encourage further development by interested users, whether that involves adding new control schemes, creating interactions between multiple robots, or modifying the actuation mechanism.

## Tendon actuation
#### Open-loop case
The logic and math behind the tendon actuation for the open-loop case is the same as is showcased in [TendonForces](https://github.com/gabotuzl/TendonForces). This is because TendonForces allows for custom tendon-driven configurations for a continuum robot. Also, it requires the specification of the tension applied to each tendon, which makes it only useful in an open-loop case.
#### Closed-loop case
The closed-loop case requires a modification of the [TendonForces](https://github.com/gabotuzl/TendonForces), as it requires some sort of feedback and subsequent processing and application of the control signal. This is a problem that benefits from the use of ROS2 to solve, as the reception of the feedback data and its processing can be delegated to a controller node in the ROS2 environment, leaving only the reception of the control signal to be applied to the simulated continuum robot. Given this alleviation, the external forcing module that is used for the closed-loop case is QuadTendonForces, which allows for the updating of the active tendons and their respective tensions.
The logic and math behind QuadTendonForces is very similar to that of TendonForces, except that the tendon configuration is fixed (there are four long tendons and four short tendons, however their parameters can be specified by the user), and tension is not specified by the user, but rather the controller node in the ROS2 environment.

## General function of tendon-cr-environment nodes
The function of the nodes will be broken down in the following sections. For a more in-depth explanation, refer to the work: (**INSERT THESIS HERE**).

### GUI node
#### Open-loop case
In the open-loop case, asdasdasd
#### Closed-loop case
In the closed-loop case, asdasdasd

### Simulator node
#### Open-loop case
In the open-loop case, asdasd
#### Closed-loop case
In the closed-loop case, asdasdasd

### Visualizer node
The visualizer node is in charge of asdasdasdasdasd

### Controller node
The controller node is used only in the closed-loop case. It is in charge of asdasdasdas

### Gain searcher node
The gain searcher node used only in the closed-loop case. It is in charge of asdaasdasdasd

## Installation and usage
asdasd

## Dependencies
Rclpy, PyElastica, Numpy, Matplotlib, Moviepy, etc.
