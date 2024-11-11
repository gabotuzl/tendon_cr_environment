#!/usr/bin/env python3
import tkinter
import tkinter.messagebox
import customtkinter
from PIL import Image

import rclpy
from rclpy.node import Node
import numpy as np
import time
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32


customtkinter.set_appearance_mode("Dark")
customtkinter.set_default_color_theme("blue")

class GUI(customtkinter.CTk):

    def __init__(self, node_object):
        super().__init__()

        # Node_object to be able to publish
        self.gui_node = node_object

        # Set the window
        self.title("Graphic User Interface for Open Loop Tendon Forcing")
        self.geometry(f"{1150}x{950}")

        # Set the grid layout
        self.grid_columnconfigure((0,1,2), weight=0)
        self.grid_rowconfigure((0), weight=0)

        # Parameters section
        self.parameters_section = customtkinter.CTkFrame(self, width=440, height=950, corner_radius=10)
        self.parameters_section.grid(column=0, row=0, sticky="nsew")
        self.parameters_section.grid_columnconfigure((0), weight=0)
        self.parameters_section.grid_rowconfigure((0, 1, 2, 3), weight=0)
        self.parameters_title = customtkinter.CTkLabel(self.parameters_section, text="Parameters", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.parameters_title.grid(row=0, column=0, padx=20, pady=20)
        self.parameters_section.grid_propagate(False)

        # Rod parameters section
        self.rod_parameters_section = customtkinter.CTkFrame(self.parameters_section, width=400, height = 346, corner_radius=10)
        self.rod_parameters_section.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        self.rod_parameters_section.grid_rowconfigure((0,1,2,3,4,5,6,7), weight=1)
        self.rod_parameters_section.grid_columnconfigure((0,1), weight=1)
        self.rod_parameters_section_title = customtkinter.CTkLabel(self.rod_parameters_section, text= "Rod Parameters",font=customtkinter.CTkFont(size=15, weight="bold") )
        self.rod_parameters_section_title.grid(row=0, column=0, padx=10, pady=10)
        self.rod_parameters_section.grid_propagate(False)

        self.var1_label = customtkinter.CTkLabel(self.rod_parameters_section, text="Original length of rod:")
        self.var1_label.grid(row=1, column=0, padx=5, pady=5, sticky="nw")
        self.var1_entry = customtkinter.CTkEntry(self.rod_parameters_section, width=100, placeholder_text='Enter value')
        self.var1_entry.grid(row=1, column=1, padx=5, pady=5, sticky="ne")

        self.var2_label = customtkinter.CTkLabel(self.rod_parameters_section, text="Original radius of rod:")
        self.var2_label.grid(row=2, column=0, padx=5, pady=5, sticky="nw")
        self.var2_entry = customtkinter.CTkEntry(self.rod_parameters_section, width=100, placeholder_text='Enter value')
        self.var2_entry.grid(row=2, column=1, padx=5, pady=5, sticky="ne")

        self.var3_label = customtkinter.CTkLabel(self.rod_parameters_section, text="Density of rod:")
        self.var3_label.grid(row=3, column=0, padx=5, pady=5, sticky="nw")
        self.var3_entry = customtkinter.CTkEntry(self.rod_parameters_section, width=100, placeholder_text='Enter value')
        self.var3_entry.grid(row=3, column=1, padx=5, pady=5, sticky="ne")

        self.var4_label = customtkinter.CTkLabel(self.rod_parameters_section, text="Young's Modulus of rod:")
        self.var4_label.grid(row=4, column=0, padx=5, pady=5, sticky="nw")
        self.var4_entry = customtkinter.CTkEntry(self.rod_parameters_section, width=100, placeholder_text='Enter value')
        self.var4_entry.grid(row=4, column=1, padx=5, pady=5, sticky="ne")

        self.var5_label = customtkinter.CTkLabel(self.rod_parameters_section, text="Shear Modulus of rod:")
        self.var5_label.grid(row=5, column=0, padx=5, pady=5, sticky="nw")
        self.var5_entry = customtkinter.CTkEntry(self.rod_parameters_section, width=100, placeholder_text='Enter value')
        self.var5_entry.grid(row=5, column=1, padx=5, pady=5, sticky="ne")

        self.var6_label = customtkinter.CTkLabel(self.rod_parameters_section, text="Damping Constant of rod:")
        self.var6_label.grid(row=6, column=0, padx=5, pady=5, sticky="nw")
        self.var6_entry = customtkinter.CTkEntry(self.rod_parameters_section, width=100, placeholder_text='Enter value')
        self.var6_entry.grid(row=6, column=1, padx=5, pady=5, sticky="ne")

        self.var7_label = customtkinter.CTkLabel(self.rod_parameters_section, text="Number of Elements (Rod Discretization):")
        self.var7_label.grid(row=7, column=0, padx=5, pady=5, sticky="nw")
        self.var7_entry = customtkinter.CTkEntry(self.rod_parameters_section, width=100, placeholder_text='Enter value')
        self.var7_entry.grid(row=7, column=1, padx=5, pady=5, sticky="ne")

        # Simulation parameters
        self.sim_parameters_section = customtkinter.CTkFrame(self.parameters_section, width=400, height=260, corner_radius=0)
        self.sim_parameters_section.grid(column=0, row=2, sticky="nsew", padx=5, pady=5)
        self.sim_parameters_section.grid_rowconfigure((0, 1, 2, 3, 4, 5), weight=1)
        self.sim_parameters_section.grid_columnconfigure((0, 1), weight=1)
        self.sim_parameters_title = customtkinter.CTkLabel(self.sim_parameters_section, text="Simulation Parameters", font=customtkinter.CTkFont(size=15,  weight="bold"))
        self.sim_parameters_title.grid(row=0, column=0, padx=10, pady=10)
        self.sim_parameters_section.grid_propagate(False)


        self.var8_label = customtkinter.CTkLabel(self.sim_parameters_section, text="Total simulation time:")
        self.var8_label.grid(row=1, column=0, padx=5, pady=5, sticky="nw")
        self.var8_entry = customtkinter.CTkEntry(self.sim_parameters_section, width=100, placeholder_text='Enter value')
        self.var8_entry.grid(row=1, column=1, padx=5, pady=5, sticky="ne")

        self.var9_label = customtkinter.CTkLabel(self.sim_parameters_section, text="Rendering FPS:")
        self.var9_label.grid(row=2, column=0, padx=5, pady=5, sticky="nw")
        self.var9_entry = customtkinter.CTkEntry(self.sim_parameters_section, width=100, placeholder_text='Enter value')
        self.var9_entry.grid(row=2, column=1, padx=5, pady=5, sticky="ne")

        self.var10_label = customtkinter.CTkLabel(self.sim_parameters_section, text="Time step:")
        self.var10_label.grid(row=3, column=0, padx=5, pady=5, sticky="nw")
        self.var10_entry = customtkinter.CTkEntry(self.sim_parameters_section, width=100, placeholder_text='Enter value')
        self.var10_entry.grid(row=3, column=1, padx=5, pady=5, sticky="ne")

        self.var11_label = customtkinter.CTkLabel(self.sim_parameters_section, text="Vertical direction (X=0, Y=1, Z=2):")
        self.var11_label.grid(row=4, column=0, padx=5, pady=5, sticky="nw")
        self.var11_entry = customtkinter.CTkEntry(self.sim_parameters_section, width=100, placeholder_text='Enter value')
        self.var11_entry.grid(row=4, column=1, padx=5, pady=5, sticky="ne")

        self.var12_label = customtkinter.CTkLabel(self.sim_parameters_section, text="Gravity magnitude (m/s^2):")
        self.var12_label.grid(row=5, column=0, padx=5, pady=5, sticky="nw")
        self.var12_entry = customtkinter.CTkEntry(self.sim_parameters_section, width=100, placeholder_text='Enter value')
        self.var12_entry.grid(row=5, column=1, padx=5, pady=5, sticky="ne")

        # Control section
        self.control_section = customtkinter.CTkFrame(self, width=400, height=950, corner_radius=10)
        self.control_section.grid(column=1, row=0, sticky='nsew')
        self.control_section_label = customtkinter.CTkLabel(self.control_section, text="Control configuration", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.control_section_label.grid(row=0, column=0, padx=20, pady=20)

        self.tendon_section = TendonFrame(self.control_section, index=1, width=400, height = 346, corner_radius=10)
        self.tendon_section.grid(row=1, column=0)

        self.desired_position_section = customtkinter.CTkFrame(self.control_section, width=340, height = 120, corner_radius=10)
        self.desired_position_section.grid(row=2, column=0, pady=10)
        self.desired_position_section.grid_columnconfigure((0, 1), weight=1)
        self.desired_position_section.grid_rowconfigure((0, 1, 2), weight=1)
        self.desired_position_section.grid_propagate(False)

        self.var13_label = customtkinter.CTkLabel(self.desired_position_section, text="Desired GLOBAL X Point (m):")
        self.var13_label.grid(row=0, column=0, padx=5, pady=5, sticky="nw")
        self.var13_entry = customtkinter.CTkEntry(self.desired_position_section, width=100, placeholder_text='Enter value')
        self.var13_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ne")

        self.var14_label = customtkinter.CTkLabel(self.desired_position_section, text="Desired GLOBAL Y Point (m):")
        self.var14_label.grid(row=1, column=0, padx=5, pady=5, sticky="nw")
        self.var14_entry = customtkinter.CTkEntry(self.desired_position_section, width=100, placeholder_text='Enter value')
        self.var14_entry.grid(row=1, column=1, padx=5, pady=5, sticky="ne")

        self.var15_label = customtkinter.CTkLabel(self.desired_position_section, text="Desired GLOBAL Z Point (m):")
        self.var15_label.grid(row=2, column=0, padx=5, pady=5, sticky="nw")
        self.var15_entry = customtkinter.CTkEntry(self.desired_position_section, width=100, placeholder_text='Enter value')
        self.var15_entry.grid(row=2, column=1, padx=5, pady=5, sticky="ne")

        self.search_checkbox_string = customtkinter.IntVar(value=0)
        self.search_checkbox = customtkinter.CTkCheckBox(self.control_section, text="Enable Gain Search", variable=self.search_checkbox_string, command=self.toggle_search_checkbox)
        self.search_checkbox.grid(row=3, column=0, pady=10, padx=10, sticky='w')

        self.search_section = customtkinter.CTkFrame(self.control_section, width=340, height = 120, corner_radius=10)
        self.search_section.grid(row=4, column=0, pady=10)
        self.search_section.grid_columnconfigure((0, 1), weight=1)
        self.search_section.grid_rowconfigure((0, 1, 2, 3), weight=1)
        self.search_section.grid_propagate(False)
        self.create_search_fields()


        # Info section
        self.info_section = customtkinter.CTkFrame(self, width=270, height=950, corner_radius=10)
        self.info_section.grid(column=2, row=0, sticky='nsew')
        self.info_section.grid_rowconfigure((0,1,2,3), weight=0)
        self.info_section.grid_columnconfigure((0), weight=0)
        self.info_section.grid_propagate(False)


        # Status display
        self.status_section = customtkinter.CTkFrame(self.info_section, width=270, height=200, corner_radius=10)
        self.status_section.grid(column=0, row=0, sticky="nsew", padx=5, pady=5)
        self.status_section.grid_rowconfigure((0,1), weight=0)
        self.status_section.grid_columnconfigure((0), weight=0)
        self.status_title = customtkinter.CTkLabel(self.status_section, text="Status", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.status_title.grid(row=0, column=0, padx=0, pady=20, sticky='nsew')
        self.status_message = customtkinter.CTkLabel(self.status_section, text="", font=customtkinter.CTkFont(size=12, weight="bold"), text_color="black")
        self.status_message.grid(row=1, column=0, padx=5, pady=5, sticky='nsew')

        # Global orientation image
        global_orientation_image_opened = Image.open("/home/gab/ros2_ws/src/mrobot_controller/mrobot_controller/images/global reference frame.png")
        self.global_orientation_image = customtkinter.CTkImage(light_image=global_orientation_image_opened, dark_image= global_orientation_image_opened, size=(200,200))
        self.global_orientation_image_label = customtkinter.CTkLabel(self.info_section, text="", image=self.global_orientation_image)
        self.global_orientation_image_label.grid(row=1, column=0, padx=5, pady=5, sticky='nsew')

        # Local orientation image
        local_orientation_image_opened = Image.open("/home/gab/ros2_ws/src/mrobot_controller/mrobot_controller/images/local reference frame.png")
        self.local_orientation_image = customtkinter.CTkImage(light_image=local_orientation_image_opened, dark_image= local_orientation_image_opened, size=(200,200))
        self.local_orientation_image_label = customtkinter.CTkLabel(self.info_section, text="", image=self.local_orientation_image)
        self.local_orientation_image_label.grid(row=2, column=0, padx=5, pady=5, sticky='nsew')

        # Button section
        self.button_section = customtkinter.CTkFrame(self.info_section, width=250, height=200, corner_radius=10)
        self.button_section.grid(column=0, row=3, sticky="nsew", pady=5)
        self.button_section.grid_rowconfigure((0, 1, 2), weight=0)
        self.button_section.grid_columnconfigure((0), weight=0)

        # Run button
        self.runbutton = customtkinter.CTkButton(self.button_section, text="RUN", fg_color="blue", border_width=2, command=self.send_parameters)
        self.runbutton.grid(row=0, column=0, padx=5, pady=5, sticky='nsew')

        # Default values button
        self.default_button = customtkinter.CTkButton(self.button_section, text="SET DEFAULT VALUES", fg_color="orange", border_width=2, command=self.default_values)
        self.default_button.grid(row=1, column=0, padx=5, pady=5, sticky='nsew')

        # Clear input button
        self.clear_button = customtkinter.CTkButton(self.button_section, text="CLEAR", fg_color="green", border_width=2, command=self.clear_input)
        self.clear_button.grid(row=2, column=0, padx=5, pady=5, sticky='nsew')

    def create_search_fields(self):
        # Clears any previous entry fields that might be present
        for widget in self.search_section.winfo_children():
            widget.destroy()

        self.search_fields = []

        if self.search_checkbox_string.get() == 0:
            # Creates fields for a proportional control without the search of the proportional gains
            labels = ["Kp Long Tendons: ", "Kp Short Tendons: "]

            # Integer that will be sent to other nodes to communicate that the gain search option is switched off
            self.search_bool = 0
        else:
            # Creates fields for the searcher to find the best proportional gains for the parameters and conditions set by the user
            labels = ["Initial Kp Long Tendons: ", "Initial Kp Short Tendons: ", "Desired time of convergence: "]

            # Integer that will be sent to other nodes to communicate that the gain search option is switched on
            self.search_bool = 1

        index = 1
        for label_text in labels:
            label = customtkinter.CTkLabel(self.search_section, text=label_text)
            label.grid(row=index, column=0, sticky='w', padx=5, pady=5)
            entry = customtkinter.CTkEntry(self.search_section, placeholder_text="Enter Value")
            entry.grid(row=index, column=1, sticky='e', padx=5, pady=5)
            self.search_fields.append(entry)
            index += 1


    def toggle_search_checkbox(self):
        self.create_search_fields()


    def send_parameters(self):
        var1 = self.var1_entry.get()    # base_length
        var2 = self.var2_entry.get()    # base_radius
        var3 = self.var3_entry.get()    # density
        var4 = self.var4_entry.get()    # youngs_modulus
        var5 = self.var5_entry.get()    # shear_modulus
        var6 = self.var6_entry.get()    # damping constant
        var7 = self.var7_entry.get()    # n_elements
        var8 = self.var8_entry.get()    # final_time
        var9 = self.var9_entry.get()    # rendering_fps
        var10 = self.var10_entry.get()  # time_step
        var11 = self.var11_entry.get()  # vertical_direction
        var12 = self.var12_entry.get()  # gravity_magnitude

        var13 = self.var13_entry.get()  # x_desired
        var14 = self.var14_entry.get()  # y_desired
        var15 = self.var15_entry.get()  # z_desired

        var20 = self.tendon_section.entries[0].get()  # vertebra_height_long
        var21 = self.tendon_section.entries[1].get()  # num_vertebrae_long
        var22 = self.tendon_section.entries[2].get()  # first_vertebra_node_long
        var23 = self.tendon_section.entries[3].get()  # final_vertebra_node_long
        var24 = self.tendon_section.entries[4].get()  # vertebra_mass_long

        var25 = self.tendon_section.entries[5].get()  # vertebra_height_short
        var26 = self.tendon_section.entries[6].get()  # num_vertebrae_short
        var27 = self.tendon_section.entries[7].get()  # first_vertebra_node_short
        var28 = self.tendon_section.entries[8].get()  # final_vertebra_node_short
        var29 = self.tendon_section.entries[9].get()  # vertebra_mass_short

        var30 = self.search_fields[0].get()     # kp_long
        var31 = self.search_fields[1].get()     # kp_short

        all_vars = []
        all_vars = [var1, var2, var3, var4, var5, var6, var7, var8, var9, var10, var11, var12, var13, var14,
                                      var15, var20, var21, var22, var23, var24, var25, var26, var27, var28, var29, var30, var31]

        if self.search_bool == 1:
            var32 = self.search_fields[2].get()     # time_desired
            all_vars.append(var32)

        if any(item == '' for item in all_vars):
            print("Error: Please fill all the fields.")
            self.status_message.configure(text="Error: Please fill all the fields.", fg_color="red")
            return

        min_time_step = float(var1)/float(var7) * np.sqrt(float(var3) / max(float(var4), float(var5))) # Specified by PyElastica

        if float(var10)>min_time_step:
            print("Error: Time step is too large.")
            self.status_message.configure(text="Error: Time step is too large.", fg_color="red")
            return

        sim_parameters = Float64MultiArray()
        sim_parameters.data = [float(var1), float(var2), float(var3), float(var4), float(var5), float(var6), float(var7), float(var8), float(var9), float(var10),
                                float(var11), float(var12)]
        self.gui_node.sim_parameters_publisher.publish(sim_parameters)
        time.sleep(0.1)

        graphing_parameters = Float64MultiArray()
        graphing_parameters.data = [float(var1), float(var7), float(var8), float(var9), float(var21), float(var22), float(var23), float(var20)]
        self.gui_node.graphing_parameters_publisher.publish(graphing_parameters)
        time.sleep(0.1)

        tendon_parameters = Float64MultiArray()
        tendon_parameters.data = [float(var20), float(var21), float(var22), float(var23), float(var24), float(var25), float(var26), float(var27), float(var28), float(var29)]
        self.gui_node.tendon_parameters_publisher.publish(tendon_parameters)
        time.sleep(0.1)

        tendon_tensions_init = Int32()
        tendon_tensions_init.data = 1
        self.gui_node.initialize_tendon_tensions_publisher.publish(tendon_tensions_init)

        desired_values = Float64MultiArray()
        desired_values.data = [float(var13), float(var14), float(var15), float(var30), float(var31), float(self.search_bool)]
        if self.search_bool == 1:
            desired_values.data.append(float(var32))
        self.gui_node.desired_values_publisher.publish(desired_values)
        time.sleep(0.1)

        self.status_message.configure(text="Processing...", fg_color="light green")

    def default_values(self):
        self.clear_input()

        var1 = self.var1_entry.insert(0, 0.25)              # base_length
        var2 = self.var2_entry.insert(0, 0.011/2)           # base_radius
        var3 = self.var3_entry.insert(0, 997.7)             # density
        var4 = self.var4_entry.insert(0, 16.598637e6)       # youngs_modulus
        var5 = self.var5_entry.insert(0, 7.216880e6)        # shear_modulus
        var6 = self.var6_entry.insert(0, 0.1)               # damping constant
        var7 = self.var7_entry.insert(0, 100.0)             # n_elements
        var8 = self.var8_entry.insert(0, 2.0)               # final_time
        var9 = self.var9_entry.insert(0, 30.0)              # rendering_fps
        var10 = self.var10_entry.insert(0, 1.8e-5)          # time_step
        var11 = self.var11_entry.insert(0, 2)               # vertical_direction
        var12 = self.var12_entry.insert(0, -9.80665)        # gravity_magnitude

        var13 = self.var13_entry.insert(0, 0.22)            # x_desired
        var14 = self.var14_entry.insert(0, 0.05)            # y_desired
        var15 = self.var15_entry.insert(0, 0.08)            # z_desired

        self.tendon_section.entries[0].insert(0, 0.015)     # vertebra_height_long
        self.tendon_section.entries[1].insert(0, 6.0)       # num_vertebrae_long
        self.tendon_section.entries[2].insert(0, 2.0)       # first_vertebra_node_long
        self.tendon_section.entries[3].insert(0, 98.0)      # final_vertebra_node_long
        self.tendon_section.entries[4].insert(0, 0.002)     # vertebra_mass_long

        self.tendon_section.entries[5].insert(0, 0.008)     # vertebra_height_short
        self.tendon_section.entries[6].insert(0, 6.0)       # num_vertebrae_short
        self.tendon_section.entries[7].insert(0, 2.0)       # first_vertebra_node_short
        self.tendon_section.entries[8].insert(0, 30.0)      # final_vertebra_node_short
        self.tendon_section.entries[9].insert(0, 0.001)     # vertebra_mass_short

        self.search_fields[0].insert(0, 3.0)                # kp_long
        self.search_fields[1].insert(0, 1.0)                # kp_short

        if self.search_bool == 1:
            self.search_fields[2].insert(0, 1.0)            # time_desired

        self.status_message.configure(text="", fg_color="gray")

    def clear_input(self):
        self.var1_entry.delete(0, 20)
        self.var2_entry.delete(0, 20)
        self.var3_entry.delete(0, 20)
        self.var4_entry.delete(0, 20)
        self.var5_entry.delete(0, 20)
        self.var6_entry.delete(0, 20)
        self.var7_entry.delete(0, 20)
        self.var8_entry.delete(0, 20)
        self.var9_entry.delete(0, 20)
        self.var10_entry.delete(0, 20)
        self.var11_entry.delete(0, 20)
        self.var12_entry.delete(0, 20)

        self.var13_entry.delete(0, 20)
        self.var14_entry.delete(0, 20)
        self.var15_entry.delete(0, 20)

        for i in range(len(self.tendon_section.entries)):
            self.tendon_section.entries[i].delete(0, 20)

        for i in range(len(self.search_fields)):
            self.search_fields[i].delete(0,20)

        self.status_message.configure(text="", fg_color="gray")

class ScrollableTendonSection(customtkinter.CTkScrollableFrame):
    def __init__(self, master, item_list, command=None, **kwargs):
        super().__init__(master, **kwargs)

class TendonFrame(customtkinter.CTkFrame):
    def __init__(self, master, index, *args, **kwargs):
        super().__init__(master, *args, **kwargs)
        self.entries = []

        text_entries = ["Vertebra height (long):", "Number of vertebrae (long):", "First vertebra node (long):", "Final vertebra node (long):", "Vertebra mass (long):",
                        "Vertebra height (short):", "Number of vertebrae (short):", "First vertebra node (short):", "Final vertebra node (short):", "Vertebra mass (short):"]

        customtkinter.CTkLabel(self, text="Configure Long and short tendons\nLong/short refers to tendons (see image)",font=customtkinter.CTkFont(size=10, weight="bold")).grid(row=0, column=0, padx=5, pady=5, sticky='w')
        for i in range(len(text_entries)):
            customtkinter.CTkLabel(self, text=text_entries[i]).grid(row=i+1, column=0, padx=5, pady=5, sticky='w')
            var_entry = customtkinter.CTkEntry(self)
            var_entry.grid(row=i+1, column=1, padx=5, pady=5)
            self.entries.append(var_entry)

        self.grid_columnconfigure((0,1), weight=1)


class gui_node(Node):
    def __init__(self):
        super().__init__("gui_control_node")
        self.get_logger().info("GUI control node has been started.")

        # Create publishers
        self.sim_parameters_publisher = self.create_publisher(Float64MultiArray, '/simulation_parameters', 10)
        self.tendon_parameters_publisher = self.create_publisher(Float64MultiArray, '/tendon_parameters', 10)
        self.graphing_parameters_publisher = self.create_publisher(Float64MultiArray, '/graphing_parameters', 10)
        self.desired_values_publisher = self.create_publisher(Float64MultiArray, '/desired_values', 10)
        self.initialize_tendon_tensions_publisher = self.create_publisher(Int32, '/tendon_tensions_init', 10)

def main():
    rclpy.init()
    node_object=gui_node()
    app = GUI(node_object)
    app.mainloop()
    try:
        rclpy.spin(node_object)
    except KeyboardInterrupt:
        pass

    node_object.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()