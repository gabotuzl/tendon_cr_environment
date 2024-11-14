# tendon_cr_environment
ROS2 environment for the simulation and control of a tendon-driven continuum robot. 

The purpose of this project is to enable users to run simulations and implement control schemes for a tendon-driven continuum robot. ROS2 Humble was utilized to construct the interactive environment, which allows for the integration of multiple functions operating in parallel and facilitates the transition to the physical implementation of the robot. The simulation of the continuum robot is performed using PyElastica (https://github.com/GazzolaLab/PyElastica). This project’s open-source nature is intended to facilitate code inspection and encourage further development by interested users, whether that involves adding new control schemes, creating interactions between multiple robots, or modifying the actuation mechanism.

## Tendon actuation
#### Open-loop case
The logic and math behind the tendon actuation for the open-loop case is the same as is showcased in [TendonForces](https://github.com/gabotuzl/TendonForces). This is because TendonForces allows for custom tendon-driven configurations for a continuum robot. Also, it requires the specification of the tension applied to each tendon, which makes it only useful in an open-loop case.
#### Closed-loop case
The closed-loop case requires a modification of the [TendonForces](https://github.com/gabotuzl/TendonForces), as it requires some sort of feedback and subsequent processing and application of the control signal. This is a problem that benefits from the use of ROS2 to solve, as the reception of the feedback data and its processing can be delegated to a controller node in the ROS2 environment, leaving only the reception of the control signal to be applied to the simulated continuum robot. 

Given this alleviation, the external forcing module that is used for the closed-loop case is QuadTendonForces, which allows for the updating of the active tendons and their respective tensions.
The logic and math behind QuadTendonForces is very similar to that of TendonForces, except that the tendon configuration is fixed (there are four long tendons and four short tendons, however their parameters can be specified by the user), and tension is not specified by the user, but rather the controller node in the ROS2 environment.

In the case where a different tendon configuration is desired by the user, a new forcing module must be created to suit that specific configuration. The purpose of QuadTendonForces's configuration is to allow for a maximum of ONE concavity change caused by pairs of antagonist tendons.

## General function of tendon-cr-environment nodes
The general function of the nodes will be broken down in the following sections. For a more in-depth explanation, refer to the work: (**INSERT THESIS HERE**).

### GUI node
All parameters specified in the GUI's are in SI units.
#### Open-loop case (GUI manual)
Provides a simple GUI for the simulation of an open-loop tendon-driven continuum robot system. The user can specify the rod's parameters, the simulation's parameters, as well as whatever custom tendon actuation configuration desired (many tendons can be added to a single system, each having custom parameters).
#### Closed-loop case (GUI control)
Provides a simple GUI for the simulation and control of a closed-loop tendon-driven continuum robot system. The user can specify the rod's parameters, the simulation's parameters, the parameters for the QuadTendonForces tendon configuration (custom parameters for the four "long" tendons and custom parameters the four "short" tendons), the desired XYZ tip position for the robot, as well as the proportional gains for the controller which are specified for the "long" tendons as well as the "short" ones. To make use of the gain searching function, this GUI has a checkbox which enables or disables the gain search function. If this function is enabled, the user can specify the time of convergence desired, which will be used to fine tune the proportional gains to reach the desired position in the time specified by the user.

### Simulator node
This node is set up in a way that when it receives the information from the GUI node, it can differentiate whether it was sent from the manual GUI or the control GUI, and thus choose which of the tendon actuation external forcing modules to use (TendonForces or QuadTendonForces).
#### Open-loop case
In a very straight-forward manner, this node receives parameters specified in the manual GUI by the user and runs an open-loop simulation, after which it publishes the results to the visualizer node to be processed and seen by the user.
#### Closed-loop case
Because the publishing and subscribing of information during the simulation is required, two extra nodes are started and used within the simulator node for these purposes and are used in the PyElastica simulator's callback function. The publishing of information is mostly for the controller node so that it can carry out the processing and control logic, and the subscribing is to update the activated tendons in the system as well as their respective tensions. Once the simulation is carried out, the results are published to the visualizer node to be processed and seen by the user.

### Visualizer node
This node receives parameters from the chosen GUI node as well as the position and directors history from the simulator node, subsequently creating a video of the evolution of the system in the alotted time (this is saved in the home directory), as well as a 3D plot of the final pose of the robot.

### Controller node
This node receives parameters from the control GUI as well as current information about the simulation taking place. It is in charge of choosing which tendons must be activated to reach the desired XYZ tip position, as well as updating the tensions assigned to each of those activated tendons. In the case that the gain searcher option is activated, it will iteratively change its proportional gains values, as specified by the gain searcher node's logic.

### Gain searcher node
This node aims to find the best proportional gains to use for the controller node in order to reach the desired XYZ tip position for the robot in the desired time specified by the user. This is done by carrying out simulations while changing the proportional gains accordingly, until a certain tolerance of time taken is reached.

## Installation and usage

### Dependencies

Ensure you have the following prerequisites installed:
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [rclpy](https://github.com/ros2/rclpy)
- [PyElastica](https://github.com/GazzolaLab/PyElastica)
- [numpy](https://numpy.org/)
- [numba](https://numba.pydata.org/)
- [matplotlib](https://matplotlib.org/)
- [moviepy](https://zulko.github.io/moviepy/)
- [customtkinter](https://github.com/TomSchimansky/CustomTkinter)
- [Pillow](https://pillow.readthedocs.io/en/stable/)

### Installation

1. **Clone the Repository**

   Clone this repository to your local machine:

   ```bash
   git clone https://github.com/gabotuzl/tendon_cr_environment.git
   cd tendon_cr_environment
   ```

2. **Build the ROS2 Workspace**

   Navigate to the `environment_ws` folder and build the workspace:

   ```bash
   cd tendon_cr_environment_ws
   colcon build
   ```

3. **Source the Workspace**

   After the build is successful, source the setup script to add the workspace to your ROS2 environment:

   ```bash
   source install/setup.bash
   ```

   You may want to add this line to your shell startup file (e.g., `~/.bashrc`) for convenience:

   ```bash
   echo "source /path/to/repo_name/tendon_cr_environment_ws/install/setup.bash" >> ~/.bashrc
   ```

### Usage

#### Running the Environment with Launch Files
To run the full simulation environment, you can use the provided launch files. 

To use the manual (open-loop) simulation environment which contemplates custom tendon configurations, open a terminal and execute:

```bash
ros2 launch tendon_cr_environment manual_tcre.launch.py
```
To use the control (closed-loop) simulation environment which allows for the control of the robot tip's position by using a predetermined tendon setup (with customizable parameters), open a terminal and execute:

```bash
ros2 launch tendon_cr_environment control_tcre.launch.py
```

#### Running Nodes Separately

If you prefer to run each node individually, you can do so by following these steps:

1. Open multiple terminal windows (or tabs).
2. Source the workspace in each terminal:

   ```bash
   source /path/to/repo_name/tendon_cr_environment_ws/install/setup.bash
   ```

3. Start each node by running the following commands in separate terminals:

   - **GUI Node(s):**
     For the open-loop simulation GUI:
     ```bash
     ros2 run tendon_cr_enviroment gui_manual_node
     ```
     For the closed-loop simulation and control GUI:
     ```bash
     ros2 run tendon_cr_environment gui_control_node
     ```

   - **Simulator Node:**

     ```bash
     ros2 run tendon_cr_environment simulator_node
     ```
     
   - **Visualizer Node:**

     ```bash
     ros2 run tendon_cr_environment visualizer_node
     ```
     
   - **Controller Node:**

     ```bash
     ros2 run tendon_cr_environment controller_node
     ```
     
   - **Gain Searcher Node:**

     ```bash
     ros2 run tendon_cr_environment gain_searcher_node
     ```



