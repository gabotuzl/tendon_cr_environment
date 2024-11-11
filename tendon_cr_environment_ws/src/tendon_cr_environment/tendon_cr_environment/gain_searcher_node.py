#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
import numpy as np
import time

from numba import njit
import threading

class GainSearcher(Node):
    def __init__(self):
        super().__init__("gain_searcher_node")
        self.get_logger().info("Searcher node has been started.")

        # Creating publishers
        self.proportional_gains_publisher = self.create_publisher(Float64MultiArray, '/proportional_gains', 10)
        self.initialize_tendon_tensions_publisher = self.create_publisher(Int32, '/init_tendon_tensions', 10)
        self.start_simulation_publisher = self.create_publisher(Int32, 'start_simulation', 10)


        # Creating subscribers
        self.desired_values_subscriber = self.create_subscription(Float64MultiArray, "/desired_values", self.desired_values_callback, 10)
        self.current_tip_pos_subscriber = self.create_subscription(Float64MultiArray, "/current_tip_position", self.tip_position_callback, 10)
        self.simulation_data_subscriber = self.create_subscription(Float64MultiArray, '/simulation_data', self.simulation_data_callback, 10)

        # Initializes boolean variables
        self.first_message_time = None
        self.check_norm_time = True
        self.end_simulation = False

        # Creates separate thread for the gain_searcher method, so that it can run separately from all the other methods in the node
        self.gain_searcher_thread = threading.Thread(target=self.gain_searcher)


    def desired_values_callback(self, msg:Float64MultiArray):
        # Receives the desired values (Tip XYZ and desired time for stabilization)
        self.x_desired = msg.data[0]
        self.y_desired = msg.data[1]
        self.z_desired = msg.data[2]
        self.init_gains = [msg.data[3], msg.data[4]]
        self.search_bool = True if msg.data[5] == 1.0 else False

        if self.search_bool == 1:
            self.time_desired = msg.data[6]
        
        # Initializes counter for searcher
        self.counter = 0

        print("Desired parameters received.")

        # Begins gain search thread
        self.gain_searcher_thread.start()
    
    def gain_searcher(self):
        if self.search_bool is True:
            print("Search has begun...")

            Kp_gains = self.simple_searcher(self.start_simulation, self.init_gains)
            print("\n\nPROPORTIONAL GAINS HAVE BEEN FOUND:")
            print(f"LONG TENDONS: {Kp_gains[0]}")
            print(f"SHORT TENDONS: {Kp_gains[1]}")
        else:
            print("Gain search setting is switched off.")
    
    def start_simulation(self, init_gains):
        # Counter for the simulations
        self.counter +=1

        # Initializes end_simulation_callback so that it is specified that a new simulation has begun
        self.end_simulation = False

        # Initializes timer for the simulations
        self.first_message_time = None
        self.current_simulation_time = 0.0
        
        # Initializes prev_norm and overshoot to be used to calculate the maximum overshoot of the tip position
        self.prev_norm = 0.0
        self.overshoot = 0.0

        # Initializes the tip_position_callback so that it will begin checking the tip position to find the time at which the desired values were met
        self.check_norm_time = True

        # Publishes an integer to /init_tendon_tensions so that the controller node can initialize with null tensions
        send_integer = Int32()
        send_integer.data = 1
        self.initialize_tendon_tensions_publisher.publish(send_integer)

        # Publishes proportional gains for controller node to use
        proportional_gains_array = Float64MultiArray()
        proportional_gains_array.data = [init_gains[0], init_gains[1]]
        self.proportional_gains_publisher.publish(proportional_gains_array)

        # Publishes an integer to /start_simulation so that the simulator node can begin the simulation
        self.start_simulation_publisher.publish(send_integer)

        while self.end_simulation is False:
            time.sleep(0.1)

        print(f"Simulation counter: {self.counter}")
        print(f"Kp Long: {init_gains[0]}", f"  Kp short: {init_gains[1]}")
        print("Time taken to reach desired XYZ: ", self.time_reached)

        if np.isnan(self.norm):
            self.norm = 1e10
        print("Norm of differences: ", self.norm)

        return self.overshoot, (self.time_desired-self.time_reached)

    def tip_position_callback(self, msg:Float64MultiArray):
    # Callback which receives XYZ tip position, which is published from the MyCallback class in the simulator node
    # This method saves the time until the desired XYZ position is reached for the first time, as well as calculates the norm of the 
    # difference between the desired and current values for the XYZ position
    # This callback executes the method, only if the gain search setting is switched on
        if self.search_bool is True:
            if self.check_norm_time is True:
                # Calculates the norm of the difference between desired and current tip position values and checks it against the desired tolerance
                tolerance = 0.01 #This tolerance is in meters (m)
                self.norm = self.norm_calculator(msg.data[0], msg.data[1], msg.data[2], self.x_desired, self.y_desired, self.z_desired)

                if self.norm < tolerance:
                    # This means that the tip XYZ has reached the desired values for the first time
                    # The time at which this occurred is saved as an attribute
                    self.time_reached = self.current_simulation_time
                    self.check_norm_time = False
                    self.check_overshoot = True
                else:
                    # This means that the tip XYZ has not / never reached the desired values, which means that a large number is set to steer the searcher away from this point
                    self.time_reached = self.current_simulation_time
            else:
                # If the robot has reached the desired position, its norm will still be calculated and updated as an attribute
                self.norm = self.norm_calculator(msg.data[0], msg.data[1], msg.data[2], self.x_desired, self.y_desired, self.z_desired)

                if self.check_overshoot is True:
                    # Stores the absolute value of the maximum overshoot, to be used for search of the gains
                    self.overshoot = max(self.norm, self.overshoot)
                    print("OVERSHOOT: ",self.overshoot)
        else:
            pass

    def simulation_data_callback(self, msg: Float64MultiArray):
        # Receives the time from the simulator node
        self.current_simulation_time = msg.data[0]

        # If msg.data[1] is 1, then the simulation has ended. This boolean is used in start_simulation, so that the method waits until the simulation is done to return values
        self.end_simulation = True if msg.data[1]==1.0 else False

    @staticmethod
    @njit(cache=True)
    def norm_calculator(x_current, y_current, z_current, x_desired, y_desired, z_desired):
        dx = abs(x_desired - x_current)
        dy = abs(y_desired - y_current)
        dz = abs(z_desired - z_current)

        norm = np.linalg.norm(np.array([dx, dy, dz]))

        return norm

    def simple_searcher(self, system_function, initial_values):
        tolerance = 0.002
        self.error_count = 0
        self.vals = 0.0

        # Initializes values to be used
        new_vals = system_function(initial_values)
        error = new_vals[1]
        new_gains = initial_values

        # Iterates until tolerance criteria is met, or if the change in results has been very small
        while abs(error) > tolerance and self.error_count < 5:
            for i in range(len(new_gains)):
                new_gains[i] = new_gains[i] - new_vals[1] * new_gains[i] - new_vals[0] * new_gains[i]

            if abs(new_vals[1] - self.vals) < 0.000001:
                self.error_count +=1
                self.vals = new_vals[1]
                print(f"Iteration has not made significant progress, tries left: {5-self.error_count}")
            else:
                self.vals = new_vals[1]
            
            new_vals = system_function(new_gains)
            error = new_vals[1]
            print(error)




        return new_gains

def main(args=None):
    rclpy.init(args=args)
    searcher_node=GainSearcher()
    try:
        rclpy.spin(searcher_node)
    except KeyboardInterrupt:
        self.gain_searcher_thread.join()
        pass

if __name__ == "__main__":
    main()