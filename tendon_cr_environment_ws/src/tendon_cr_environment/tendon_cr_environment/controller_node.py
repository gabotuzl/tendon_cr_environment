#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
import numpy as np

class Controller(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.get_logger().info("Controller node has been started.")

        # Creating publishers
        self.tendon_tensions_publisher = self.create_publisher(Float64MultiArray, "/tendon_tensions", 10)
        self.start_simulation_publisher = self.create_publisher(Int32, 'start_simulation', 10)

        # Creating subscribers
        self.desired_values_subscriber = self.create_subscription(Float64MultiArray, "/desired_values", self.desired_values_callback, 10)
        self.simulation_parameters_subscriber = self.create_subscription(Float64MultiArray, "/simulation_parameters", self.parameters_callback, 10)
        self.current_tip_pos_subscriber = self.create_subscription(Float64MultiArray, "/current_tip_position", self.controller_callback, 10)
        self.initialize_tendon_tensions_subsriber = self.create_subscription(Int32, "/init_tendon_tensions", self.initialize_tendon_tensions, 10)
        self.proportional_gains_subscriber = self.create_subscription(Float64MultiArray, '/proportional_gains', self.proportional_gains_callback, 10)


        # Initializing tendon tensions
        self.tension_vertical_long = 0
        self.tension_horizontal_long = 0
        self.tension_vertical_short = 0
        self.tension_horizontal_short = 0

        # Initializing gain search boolean
        self.search_bool = None

    def initialize_tendon_tensions(self, msg: Int32):
        #Initializing tendon tensions
        self.tension_vertical_long = 0.0
        self.tension_horizontal_long = 0.0
        self.tension_vertical_short = 0.0
        self.tension_horizontal_short = 0.0

        self.current_X = self.base_length
        self.current_y = 0.0
        self.current_z = 0.0

    def desired_values_callback(self, msg: Float64MultiArray):
        # Note: msg.data[6] is the desired time, which is handled by the gain_searcher_node
        # Receieves the desired XYZ position in the global reference frame, as well as proportional gains 
        self.x_desired = msg.data[0] 
        self.y_desired = msg.data[1] 
        self.z_desired = msg.data[2] 
        self.kp_long = msg.data[3]
        self.kp_short = msg.data[4]
        self.search_bool = True if msg.data[5] == 1.0 else False

        if self.search_bool == False:
            send_integer = Int32()
            send_integer.data = 1
            self.start_simulation_publisher.publish(send_integer)

        
        # Setting limits for the desired points
        if self.x_desired < 0.15:
            print("Error: Out of range for the robot.")
            return
        if self.y_desired > 0.15:
            print("Error: Out of range for the robot.")
            return
        if self.z_desired > 0.15:
            print("Error: Out of range for the robot.")
            return

    def parameters_callback(self, msg: Float64MultiArray):
        # Rod parameters
        self.base_length = msg.data[0]
        self.base_radius = msg.data[1]
        self.density = msg.data[2]
        self.youngs_modulus = msg.data[3]
        self.shear_modulus = msg.data[4]
        self.damping_constant = msg.data[5]
        self.n_elements = int(msg.data[6])

        # Motor(s) parameters
        self.motor_mass = msg.data[9]

        # Simulation parameters
        self.vertical_direction = int(msg.data[10])
        self.acc_gravity = np.zeros(3)
        self.gravity_magnitude = msg.data[11]
        self.acc_gravity[self.vertical_direction] = self.gravity_magnitude

        # Initializing counter
        self.counter = 0

    def proportional_gains_callback(self, msg:Float64MultiArray):
        if self.search_bool is True:
            # Receives and stores the proportional gains sent through /proportional_gains
            self.kp_long = msg.data[0]
            self.kp_short = msg.data[1]
        else:
            print("Gain search setting is switched off.")
    
    def controller_callback(self, msg: Float64MultiArray):

        print("\nCONTROLLER CALLBACK ACTIVATED, TIP POS RECEIVED: ")
        print(f"X: {msg.data[0]}, Y: {msg.data[1]}, Z: {msg.data[2]}")
        # tendon_tensions is organized as follows:
        # [VLT sign, HLT, sign, VST sign, HST sign, VLT tension, HLT tension, VST tension, HST tension]
        # Where, VLT = Vertical long tendon, HLT = Horizontal long tendon, VST = Vertical short tendon, HST = Horizontal short tendon
        tendon_tensions = []

        # Determines the signs of the tendons. Basically, the logic below chooses the orientation of the tendons, according to where the desired tip point is
        # This logic uses the global reference frame
        z_desired_sign = np.sign(self.z_desired)
        y_desired_sign = np.sign(self.y_desired)

        tendon_tensions.append(z_desired_sign)   #VLT
        tendon_tensions.append(y_desired_sign)   #HLT
        tendon_tensions.append(-z_desired_sign)  #VST
        tendon_tensions.append(-y_desired_sign)  #HST

        # Extracting data sent from the simulator node    
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]
        self.current_z = msg.data[2]

        # For general purposes, the Y and Z coordinate will be mostly affected by the vertical and horizontal long tendons
        # The short tendons are antagonists to the long tendons and will be used to adjust the X coordinate

        dx = (self.current_x - self.x_desired) 
        dy = (self.current_y - self.y_desired) * y_desired_sign
        dz = (self.current_z - self.z_desired) * z_desired_sign

        self.tension_vertical_long = self.positive_or_zero(self.tension_vertical_long - (dz) * self.kp_long)
        self.tension_horizontal_long = self.positive_or_zero(self.tension_horizontal_long - (dy) * self.kp_long)
        
        self.tension_vertical_short = self.positive_or_zero(self.tension_vertical_short + (dx+dz) * self.kp_short)
        self.tension_horizontal_short = self.positive_or_zero(self.tension_horizontal_short + (dx+dy) * self.kp_short)

        tendon_tensions.append(self.tension_vertical_long)
        tendon_tensions.append(self.tension_horizontal_long)
        tendon_tensions.append(self.tension_vertical_short)
        tendon_tensions.append(self.tension_horizontal_short)

        tendon_tensions_array = Float64MultiArray()
        tendon_tensions_array.data = tendon_tensions

        print("TENDON TENSIONS PUBLISHED TO TENDON FORCING: ")
        print(tendon_tensions)
        print("PROPORTIONAL GAINS USED: ")
        print("LONG TENDONS: ",self.kp_long, " SHORT TENDONS: ", self.kp_short)

        self.tendon_tensions_publisher.publish(tendon_tensions_array)

    def positive_or_zero(self, value):
        if value <= 0:
            return 0.0
        else:
            return value
        

def main(args=None):
    rclpy.init(args=args)
    controller_node=Controller()
    try:
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
