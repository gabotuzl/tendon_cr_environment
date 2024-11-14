# tendon_cr_environment

## Overview

The **tendon_cr_environment** is a ROS2-based environment designed for the simulation and control of a tendon-driven continuum robot. This project enables users to run simulations and implement control schemes for the robot. Built on ROS2 Humble, the interactive environment supports the integration of multiple functions operating in parallel, facilitating a smooth transition to physical robot implementation.

The simulation of the continuum robot leverages [PyElastica](https://github.com/GazzolaLab/PyElastica) for its backend processing. The open-source nature of this project encourages code inspection and further development by users, whether that involves adding new control schemes, creating interactions between multiple robots, or modifying the actuation mechanisms.

## Tendon Actuation

### Open-loop Case

The logic and mathematics behind tendon actuation in the open-loop case are similar to those described in [TendonForces](https://github.com/gabotuzl/TendonForces), which supports custom tendon-driven configurations for a continuum robot and requires the specification of the tension applied to each tendon, making it suitable only for open-loop control.

### Closed-loop Case

The closed-loop scenario necessitates modifications to the [TendonForces](https://github.com/gabotuzl/TendonForces) to accommodate feedback processing and the application of control signals. ROS2 simplifies this task by allowing feedback reception and processing to be handled by a controller node, while only the control signals must be applied to the simulated continuum robot.

For the closed-loop case, we utilize the **QuadTendonForces** module, which allows for active tendon updates and tension adjustments. The logic and mathematics of QuadTendonForces closely resemble those of TendonForces, differing mainly in that the tendon configuration is fixed with four long tendons and four short tendons. User-specified parameters adjust the parameters of these tendons, but tension is managed by the controller node in the ROS2 environment.

If users desire a different tendon configuration, a new forcing module must be created to cater to that specific setup. The QuadTendonForces module is designed to allow for a maximum of one concavity change caused by pairs of antagonist tendons.

## Functionality of tendon_cr_environment Nodes

The functionality of the nodes in this project is described in the following sections. For a more in-depth overview, please refer to: **INSERT THESIS HERE**.

### GUI Node

All parameters specified in the GUI are in SI units.

#### Open-loop Case (GUI Manual)

This node provides a user-friendly GUI for simulating an open-loop tendon-driven continuum robot system. Users can specify the parameters for the rod, simulation, and a custom tendon actuation configuration, with the ability to add multiple tendons, each having its own custom parameters.

#### Closed-loop Case (GUI Control)

This node facilitates the simulation and control of a closed-loop tendon-driven continuum robot system. Users can specify the rod's parameters, simulation parameters, and QuadTendonForces configuration parameters for the long and short tendons. Additionally, users can define the desired XYZ tip position for the robot and set the proportional gains for the controller. The GUI includes a gain search function, allowing users to specify a desired convergence time, which is then used to fine-tune the proportional gains for achieving the desired position within the specified time.

### Simulator Node

The simulator node intelligently processes input from either the manual GUI or the control GUI, selecting the appropriate tendon actuation external forcing module (TendonForces or QuadTendonForces) accordingly.

#### Open-loop Case

In this straightforward setup, the simulator node receives parameters from the manual GUI, runs an open-loop simulation, and publishes the results to the visualizer node for user review.

#### Closed-loop Case

The closed-loop simulation requires additional nodes for publishing and subscribing to data. Two extra nodes are initiated for this purpose, which are used within the PyElastica simulator's callback function. The published information primarily supports the controller node, while the subscriptions update the activated tendons in the system and their respective tensions. After the simulation, results are published to the visualizer node for processing.

### Visualizer Node

The visualizer node receives parameters from the chosen GUI node, along with position and direction data from the simulator node. It subsequently creates a video of the system's evolution over time (saved to the home directory) and generates a 3D plot of the robot's final pose.

### Controller Node

This node receives parameters from the control GUI and current simulation information. It determines which tendons should be activated to achieve the desired XYZ tip position and updates the tensions assigned to the activated tendons. If the gain searcher option is activated, it iteratively adjusts the proportional gains based on logic defined by the gain searcher node.

### Gain Searcher Node

The gain searcher node's goal is to identify optimal proportional gains for the controller node, to reach the desired XYZ tip position within the specified time frame. This is accomplished through simulations that vary the proportional gains until the required time tolerance is met.

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



