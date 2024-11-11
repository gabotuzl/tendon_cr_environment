#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32

from elastica.rod.cosserat_rod import CosseratRod
from elastica.boundary_conditions import OneEndFixedBC
from elastica.dissipation import AnalyticalLinearDamper
from elastica.callback_functions import CallBackBaseClass
from elastica.timestepper.symplectic_steppers import PositionVerlet
from elastica.timestepper import integrate
from elastica.external_forces import GravityForces

from elastica.modules import (
    BaseSystemCollection,
    Constraints,
    Forcing,
    CallBacks,
    Damping
)
import numpy as np
from collections import defaultdict
import asyncio
import time

# TendonForces and QuadTendonForces are put in a different file from elastica.external_forces to show it does not come by default in the Elastica simulation package
# The package's directory must be added to the path, so that it can find Tendon_Forcing and import it
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from Tendon_Forcing import TendonForces 
from Tendon_Forcing import QuadTendonForces


class Simulator(Node):

    def __init__(self, node_object1, node_object2):
        super().__init__("simulator_node")
        self.get_logger().info("Simulator node has been started.")

        # Creating publishers 
        self.position_history_publisher = self.create_publisher(Float64MultiArray, "/position_history",10)
        self.director_history_publisher = self.create_publisher(Float64MultiArray, "/director_history",10)

        # Creating subscribers
        self.simulation_parameters_subscriber = self.create_subscription(Float64MultiArray, "/simulation_parameters", self.simulation_parameters_callback, 10)
        self.tendon_parameters_subscriber = self.create_subscription(Float64MultiArray, '/tendon_parameters', self.tendon_parameters_callback, 10)
        self.start_simulation_subscriber = self.create_subscription(Int32, 'start_simulation', self.start_simulation, 10)

        # Passes the tendon_tensions_subscriber_node instance as an attribute, to be used later
        self.tendon_tensions_subscriber = node_object1

        # Passes the tip_position_publisher_node instance as an attribute, to be used later
        self.tip_position_publisher = node_object2


    async def rod_simulation(self):

        #Creating the rod
        rod1 = CosseratRod.straight_rod(
            n_elements = self.n_elements,                       # Number of elements
            start = self.start,                                 # Starting position of first node in rod
            direction = self.direction,                         # Direction the rod extends
            normal = self.normal,                               # Normal vector of rod
            base_length = self.base_length,                     # Original length of rod (m)
            base_radius = self.base_radius,                     # Original radius of rod (m)
            density = self.density,                             # Density of rod (kg/m^3)
            youngs_modulus = self.youngs_modulus,               # Elastic Modulus (Pa)
            shear_modulus = self.shear_modulus,                 # Shear Modulus (Pa)
        )
        self.CantileverRod = CantileverRodSimulator()

        # Add rod to simulator
        self.CantileverRod.append(rod1)

        # Constrain rod (Will only use OneEndFixedBC at the base)
        self.CantileverRod.constrain(rod1).using(
            OneEndFixedBC,                  # Displacement BC being applied
            constrained_position_idx=(0,),  # Node number to apply BC
            constrained_director_idx=(0,)   # Element number to apply BC
        )

        # Add forcing due to gravity (the whole rod)
        self.CantileverRod.add_forcing_to(rod1).using(
            GravityForces,
            acc_gravity = self.acc_gravity
        )
           
        # Damping on rod
        self.CantileverRod.dampen(rod1).using(
            AnalyticalLinearDamper,
            damping_constant = self.damping_constant,
            time_step = self.time_step
        )
        if self.manual_gui == True:
            # Custom tendon configuration used in conjunction with GUI_manual
            for i in range(len(self.tendon_data)):
                self.CantileverRod.add_forcing_to(rod1).using(
                TendonForces,
                vertebra_height = self.tendon_data[i][0],
                num_vertebrae = int(self.tendon_data[i][1]),
                first_vertebra_node = int(self.tendon_data[i][2]),
                final_vertebra_node = int(self.tendon_data[i][3]),
                vertebra_mass = self.tendon_data[i][4],
                tension = self.tendon_data[i][5],
                vertebra_height_orientation = np.array([self.tendon_data[i][6], self.tendon_data[i][7], self.tendon_data[i][8]]), # Orientation in the local frame (X Y Z)
                n_elements = self.n_elements
                )
        elif self.control_gui == True:
            # Quad Tendon Configuration used in conjunction with GUI_control
                self.CantileverRod.add_forcing_to(rod1).using(
                QuadTendonForces,
                vertebra_height_long = self.tendon_data[0],
                num_vertebrae_long = int(self.tendon_data[1]),
                first_vertebra_node_long = int(self.tendon_data[2]),
                final_vertebra_node_long = int(self.tendon_data[3]),
                vertebra_mass_long = self.tendon_data[4],
                vertebra_height_short = self.tendon_data[5],
                num_vertebrae_short = int(self.tendon_data[6]),
                first_vertebra_node_short = int(self.tendon_data[7]),
                final_vertebra_node_short = int(self.tendon_data[8]),
                vertebra_mass_short = self.tendon_data[9],
                n_elements = self.n_elements
                )

        # Create dictionary to hold data from callback function
        callback_data_rod1= defaultdict(list)
         
        # Add MyCallBack to SystemSimulator, telling it how often to save data (step_skip) as well as publish it
        # MyCallBack will take objects (nodes) as arguments to later access their methods (subcription and publication)
        self.CantileverRod.collect_diagnostics(rod1).using(
            MyCallBack, self, self.tendon_tensions_subscriber, self.tip_position_publisher, self.control_gui, self.total_steps, step_skip=self.step_skip, callback_params=callback_data_rod1)

        self.CantileverRod.finalize()

        timestepper = PositionVerlet()
        integrate(timestepper, self.CantileverRod, self.final_time, self.total_steps)

        position_history = callback_data_rod1["position"]   # Position data to be exported to Visualizer node
        director_history = callback_data_rod1["directors"]  # Director data to be exported to Visualizer node

        # Prints final tip position in the terminal
        self.get_logger().info("Final Tip X: " + str(position_history[-1][-3][-1]))
        self.get_logger().info("Final Tip Y: " + str(position_history[-1][-2][-1]))
        self.get_logger().info("Final Tip Z: " + str(position_history[-1][-1][-1]))
        
        # Formats the history data to be used by a ROS2 topic
        position_history_array = Float64MultiArray()
        position_history_array.data = np.array(position_history).flatten().tolist()

        director_history_array = Float64MultiArray()
        director_history_array.data = np.array(director_history).flatten().tolist()

        # History data is sent to the Visualizer node
        self.director_history_publisher.publish(director_history_array)
        time.sleep(0.1)
        self.position_history_publisher.publish(position_history_array)      
        

    def simulation_parameters_callback(self, msg: Float64MultiArray):
        # Stores and calculates values to be saved as attributes of the simulator node object

        # Rod parameters
        self.base_length = msg.data[0]
        self.base_radius = msg.data[1]
        self.density = msg.data[2]
        self.youngs_modulus = msg.data[3]
        self.shear_modulus = msg.data[4]
        self.damping_constant = msg.data[5]
        self.n_elements = int(msg.data[6])
        self.start = np.array([0.0, 0.0, 0.0])
        self.direction = np.array([1.0, 0.0, 0.0])
        self.normal = np.array([0.0, 0.0, 1.0])

        # Simulation parameters
        self.final_time = msg.data[7]
        self.rendering_fps = msg.data[8]
        self.time_step = msg.data[9]
        self.vertical_direction = int(msg.data[10])
        self.gravity_magnitude = msg.data[11]
        self.total_steps = int(self.final_time / self.time_step)
        self.step_skip = int(1.0 / (self.rendering_fps * self.time_step))

        self.acc_gravity = np.zeros(3)
        self.acc_gravity[self.vertical_direction] = self.gravity_magnitude

    def tendon_parameters_callback(self, msg: Float64MultiArray):
        # Checks whether GUI_manual or GUI_control was used, and assigns boolean variable to be used in simulator
        self.manual_gui = False
        self.control_gui = False

        if len(msg.data) % 9 == 0: # Divisible by 9 means that GUI_manual was used (9 parameters per tendon)

            # Data arrives flattened in a Float64MultiArray, and must be unflattened so that it is separated by tendons
            split = int(len(msg.data) / 9)    # 9 is the amount of data values per tendon
            self.tendon_data = np.reshape(msg.data, (split,9))
            self.manual_gui = True

            # Begins simulation after parameters are received
            asyncio.run(self.rod_simulation())

        elif len(msg.data) % 10 == 0: # GUI_control uses 10 parameters for the tendon configuration

            split = int(len(msg.data) / 10)    # 10 is the amount of data values per tendon configuration for GUI_control
            self.tendon_data = msg.data
            self.control_gui = True

        else:
            print("Error: Tendon data not sent properly.")
            return

    def start_simulation(self, msg: Int32):
        # Begins the simulation
        # This is a separate function so that it can be called as a callback to the simulation_start_subscriber
        asyncio.run(self.rod_simulation())
        

            
class CantileverRodSimulator(
    BaseSystemCollection,
    Constraints, # Enabled to use boundary conditions 'OneEndFixedBC'
    Forcing,     # Enabled to use forcing 'GravityForces', 'TendonForces', and 'QuadTendonForces'
    CallBacks,   # Enabled to use callback to collect and send data during and after the simulation
    Damping,     # Enabled to use damping models on systems.
):
    pass

class tip_position_publisher_node(Node):
    # This node is used for publishing data to the network, while being within the MyCallback object created in the simulator
     def __init__(self):
        super().__init__("tip_position_publisher_node")
        self.publisher = self.create_publisher(Float64MultiArray, "/current_tip_position", 10)
        self.simulation_data_publisher = self.create_publisher(Float64MultiArray, '/simulation_data', 10)

class tendon_tensions_subscriber_node(Node):
    # This node receives data sent by the controller node and updates the tensions in the tendons being used in the current simulation. It is only active when the GUI_control is used.
    def __init__(self):
        super().__init__("tendon_tensions_subscriber_node")
        self.subscriber = self.create_subscription(Float64MultiArray, "/tendon_tensions", self.tendon_tensions_callback, 10)


        # Initializing all the attributes defined in the controller_callback, so that PyElastica can run even if the controller has not yet sent the parameters
        self.vertebra_height_vector_vertical_long = 0.0
        self.vertebra_height_vector_horizontal_long = 0.0
        self.vertebra_height_vector_vertical_short = 0.0
        self.vertebra_height_vector_horizontal_short = 0.0

        self.tension_vertical_long = 0.0
        self.tension_horizontal_long = 0.0
        self.tension_vertical_short = 0.0
        self.tension_horizontal_short = 0.0

        self.tendon_tensions = [self.vertebra_height_vector_vertical_long, self.vertebra_height_vector_horizontal_long, self.vertebra_height_vector_vertical_short,
                                self.vertebra_height_vector_horizontal_short, self.tension_vertical_long, self.tension_horizontal_long, self.tension_vertical_short,
                                self.tension_horizontal_short]
    
    def tendon_tensions_callback(self, msg:Float64MultiArray):
        # Getting the control signal published by the control node and received through the '/tendon_tensions' topic
        try:
            self.tendon_tensions = msg.data
        except Exception as e:
            self.get_logger().error(f"Exception in tendon tensions callback spin: {e}")
            print(f"Exception in tendon tensions callback spin: {e}")
            pass
        

# MyCallBack class is derived from the base call back class. This is for position data collection, and sending tip_position to controller node and gain search node
class MyCallBack(CallBackBaseClass, Node):
    def __init__(self, simulator_node_instance, tendon_tensions_subscriber_instance, tip_position_publisher_instance, control_bool, total_steps, step_skip: int, callback_params):
        CallBackBaseClass.__init__(self)
        self.every = step_skip
        self.callback_params = callback_params
        self.total_steps = total_steps

        # Takes the three node objects as arguments to be able to access their attributes and methods
        self.simulator_node_instance = simulator_node_instance
        self.tendon_tensions_subscriber_instance = tendon_tensions_subscriber_instance
        self.tip_position_publisher = tip_position_publisher_instance

        # Attribute to determine whether or not the GUI_control was used
        self.control_bool = control_bool

        # Initializing attribute
        self.signal = Float64MultiArray()
        self.signal.data = [0.0, 0.0]

        
    # This function is called every time step
    def make_callback(self, system, time, current_step: int):

        # CantileverRod._ext_forces_torques[1] will return the tuple: (0, <external_forces.QuadTendonForces object at 0x7f353669a6b0>) (0x... will be different for each run)
        # Of course, this is because in this case, QuadTendonForces is called second to GravityForces in the forcing modules
        # This tuple must be accessed to obtain the second item, which is the instance of the forcing class QuadTendonForces
        # Once this object is accessed, its update_tendon_tension() method is called, giving the argument of 'tendon_tensions' which is
        # just the parameters obtained from the controller_node. This way, the object is updated while the simulation is taking place.

        if time>0 and self.control_bool is True:
            # Operation described in the comments above
            self.simulator_node_instance.CantileverRod._ext_forces_torques[1][1].update_tendon_tension(self.tendon_tensions_subscriber_instance.tendon_tensions)

            # Publishes the simulation time to the gain searcher node
            self.signal.data[0] = time
            self.tip_position_publisher.simulation_data_publisher.publish(self.signal)

        # If the simulation has reached its last step, then a message is published to /end_simulation, which will trigger a callback in the gain_searcher_node
        if current_step == (self.total_steps - 1):
            self.signal.data[0] = time
            self.signal.data[1] = 1.0
            self.tip_position_publisher.simulation_data_publisher.publish(self.signal)

        if current_step % self.every == 0:
            # Collects rod position data and director data for all nodes
            self.callback_params["position"].append(system.position_collection.copy())
            self.callback_params["directors"].append(system.director_collection.copy())

            # Publishes tip XYZ position to /current_tip_position topic
            tip_pos_array = Float64MultiArray()
            tip_pos_array.data = [system.position_collection[0][-1], system.position_collection[1][-1], system.position_collection[2][-1]]
            self.tip_position_publisher.publisher.publish(tip_pos_array)
            return

        

def main(args=None):
    rclpy.init(args=args)
    
    subscriber_tendon_tensions = tendon_tensions_subscriber_node()
    publisher_tip_position = tip_position_publisher_node()
    simulator_node=Simulator(subscriber_tendon_tensions, publisher_tip_position)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(simulator_node)
    executor.add_node(subscriber_tendon_tensions)

    loop = asyncio.get_event_loop()


    try:
        # Running the executor in a thread
        def spin_executor():
            # Executor spinning started
            executor.spin()

        loop.run_in_executor(None, spin_executor)
        # Asyncio event loop running
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # Shutting down nodes and executor
        executor.shutdown()
        simulator_node.destroy_node()
        subscriber_tendon_tensions.destroy_node()
        rclpy.shutdown()
        loop.stop()
        # Nodes and executor shut down completed



if __name__ == "__main__":
    main()