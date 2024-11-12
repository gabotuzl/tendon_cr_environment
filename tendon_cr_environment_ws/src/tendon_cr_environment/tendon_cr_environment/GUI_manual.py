#!/usr/bin/env python3
import tkinter
import tkinter.messagebox
import customtkinter
from PIL import Image

import rclpy
from rclpy.node import Node
import numpy as np
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


        # Tendon section
        self.tendon_section = ScrollableTendonSection(self, item_list= (1,2,3), width=400, height=950, corner_radius=10)
        self.tendon_section.grid(column=1, row=0, sticky='nsew')
        self.tendon_section = customtkinter.CTkLabel(self.tendon_section, text="Tendon Configuration", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.tendon_section.grid(row=0, column=0, padx=20, pady=20)

        self.tendon_frames=[]
        self.add_tendon_frame()

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

        # Procedure for adding the images in the GUI
        import os
        from ament_index_python import get_package_share_directory

        package_name = 'tendon_cr_environment'  # Package name
        global_image_filename = 'images/global_reference_frame.png'
        local_image_filename = 'images/local_reference_frame.png'

        # Getting the share directory of the package
        package_share_dir = get_package_share_directory(package_name)

        # Building the full path to the image
        image_path_global = os.path.join(package_share_dir, global_image_filename)
        image_path_local = os.path.join(package_share_dir, local_image_filename)

        # Global orientation image
        global_orientation_image_opened = Image.open(image_path_global)
        self.global_orientation_image = customtkinter.CTkImage(light_image=global_orientation_image_opened, dark_image= global_orientation_image_opened, size=(200,200))
        self.global_orientation_image_label = customtkinter.CTkLabel(self.info_section, text="", image=self.global_orientation_image)
        self.global_orientation_image_label.grid(row=1, column=0, padx=5, pady=5, sticky='nsew')

        # Local orientation image
        local_orientation_image_opened = Image.open(image_path_local)
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

    def add_tendon_frame(self):
        # Method that adds a new tendon to the tendon section, with the use of a button
        index = len(self.tendon_frames)+1
        tendon_frame = TendonFrame(self.tendon_section, index, width=400, height = 346, corner_radius=10)
        tendon_frame.grid(row=index+1, column=0)
        self.tendon_frames.append(tendon_frame)

        if hasattr(self, 'add_tendon_frame_button'):
            self.add_tendon_frame_button.destroy()

        self.add_tendon_frame_button = customtkinter.CTkButton(self.tendon_section, text="Add new tendon", command=self.add_tendon_frame)
        self.add_tendon_frame_button.grid(row=index+2, column=0, sticky='nsew', pady=5)

        if hasattr(self, 'remove_tendon_frame_button'):
            self.remove_tendon_frame_button.destroy()

        self.remove_tendon_frame_button = customtkinter.CTkButton(self.tendon_section, text="Remove last tendon", command=self.remove_tendon_frame, fg_color='#FF1B1B', hover_color='#FF5656')
        self.remove_tendon_frame_button.grid(row=index+3, column=0, sticky="nsew", pady=5)

    def remove_tendon_frame(self):
        # Method that removes the last tendon from the tendon section, with the use of a button
        last_tendon = self.tendon_frames.pop()
        last_tendon.destroy()

        if hasattr(self, 'add_tendon_frame_button'):
            self.add_tendon_frame_button.destroy()
        if hasattr(self, 'remove_tendon_frame_button'):
            self.remove_tendon_frame_button.destroy()

        index = len(self.tendon_frames)+1

        if self.tendon_frames:

            self.add_tendon_frame_button = customtkinter.CTkButton(self.tendon_section, text="Add new tendon", command=self.add_tendon_frame)
            self.add_tendon_frame_button.grid(row=index+2, column=0, sticky='nsew', pady=5)

            self.remove_tendon_frame_button = customtkinter.CTkButton(self.tendon_section, text="Remove last tendon", command=self.remove_tendon_frame, fg_color='#FF1B1B', hover_color='#FF5656')
            self.remove_tendon_frame_button.grid(row=index+3, column=0, sticky="nsew", pady=5)
        else:
            self.add_tendon_frame_button = customtkinter.CTkButton(self.tendon_section, text="Add new tendon", command=self.add_tendon_frame)
            self.add_tendon_frame_button.grid(row=index+2, column=0, sticky='nsew', pady=5)


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

        tendon_data = []
        current_tendon = np.zeros((9,1))
        for i in range(len(self.tendon_frames)):
            vertebra_height = self.tendon_frames[i].entries[0].get()
            num_vertebrae = self.tendon_frames[i].entries[1].get()
            first_vertebra_node = self.tendon_frames[i].entries[2].get()
            final_vertebra_node = self.tendon_frames[i].entries[3].get()
            vertebra_mass = self.tendon_frames[i].entries[4].get()
            tension = self.tendon_frames[i].entries[5].get()
            vertebra_height_orientation = self.tendon_frames[i].entries[6].get()

            current_tendon=[vertebra_height, num_vertebrae, first_vertebra_node, final_vertebra_node, vertebra_mass, tension]

            vertebra_height_orientation = vertebra_height_orientation.replace(' ','')
            vertebra_height_orientation_float_list = vertebra_height_orientation.split(',')
            if len(vertebra_height_orientation)<2:
                vertebra_height_orientation_float_list = ['']
                self.status_message.configure(text="Please fill vertebra\norientation correctly.", fg_color="red")
                print("Please fill vertebra orientation correctly.")
                return
            elif not any(item == '' for item in current_tendon):
                vertebra_height_orientation_float_list = [float(num) for num in vertebra_height_orientation_float_list]

                current_tendon = [float(vertebra_height), float(num_vertebrae), float(first_vertebra_node), float(final_vertebra_node), float(vertebra_mass), float(tension),
                                vertebra_height_orientation_float_list[0], vertebra_height_orientation_float_list[1], vertebra_height_orientation_float_list[2]]
                tendon_data.append(current_tendon)


        if any(item == '' for item in [var1, var2, var3, var4, var5, var6, var7, var8, var9, var10, var11, var12, current_tendon[0], current_tendon[1],
                                    current_tendon[2], current_tendon[3], current_tendon[4], current_tendon[5], current_tendon[6]]):
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

        tendon_parameters = Float64MultiArray()
        tendon_parameters.data = np.array(tendon_data).flatten().tolist()
        self.gui_node.tendon_parameters_publisher.publish(tendon_parameters)

        graphing_parameters = Float64MultiArray()
        graphing_parameters.data = [float(var1), float(var7), float(var8), float(var9), tendon_data[0][1], tendon_data[0][2], tendon_data[0][3], tendon_data[0][0]]
        self.gui_node.graphing_parameters_publisher.publish(graphing_parameters)

        self.status_message.configure(text="Processing...", fg_color="light green")

    def default_values(self):
        self.clear_input()
        self.add_tendon_frame()

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

        self.tendon_frames[0].entries[0].insert(0, 0.015)   # vertebra_height
        self.tendon_frames[0].entries[1].insert(0, 6.0)     # num_vertebrae
        self.tendon_frames[0].entries[2].insert(0, 2.0)     # first_vertebra_node
        self.tendon_frames[0].entries[3].insert(0, 98.0)    # final_vertebra_node
        self.tendon_frames[0].entries[4].insert(0, 0.002)   # vertebra_mass
        self.tendon_frames[0].entries[5].insert(0, 4.5)     # tension
        self.tendon_frames[0].entries[6].insert(0, '1.0,0.0,0.0') # vertebra_height_orientation

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

        for i in range(len(self.tendon_frames)):
            last_tendon = self.tendon_frames.pop()
            last_tendon.destroy()

        if hasattr(self, 'remove_tendon_frame_button'):
            self.remove_tendon_frame_button.destroy()

        self.status_message.configure(text="", fg_color="gray")

class ScrollableTendonSection(customtkinter.CTkScrollableFrame):
    def __init__(self, master, item_list, command=None, **kwargs):
        super().__init__(master, **kwargs)

class TendonFrame(customtkinter.CTkFrame):
    def __init__(self, master, index, *args, **kwargs):
        super().__init__(master, *args, **kwargs)
        self.entries = []

        text_entries = ["Vertebra height:", "Number of vertebrae:", "First vertebra node:", "Final vertebra node:", "Vertebra mass:", "Tendon tension:", "Vertebra orientation:\n(in local frame)"]

        customtkinter.CTkLabel(self, text=f"Tendon {index}",font=customtkinter.CTkFont(size=15, weight="bold")).grid(row=0, column=0, padx=5, pady=5, sticky='w')
        for i in range(len(text_entries)):
            customtkinter.CTkLabel(self, text=text_entries[i]).grid(row=i+1, column=0, padx=5, pady=5, sticky='w')
            var_entry = customtkinter.CTkEntry(self)
            var_entry.grid(row=i+1, column=1, padx=5, pady=5)
            self.entries.append(var_entry)

        self.grid_columnconfigure((0,1), weight=1)


class gui_node(Node):
    def __init__(self):
        super().__init__("gui_manual_node")
        self.get_logger().info("GUI manual node has been started.")

        # Create publishers
        self.sim_parameters_publisher = self.create_publisher(Float64MultiArray, '/simulation_parameters', 10)
        self.tendon_parameters_publisher = self.create_publisher(Float64MultiArray, '/tendon_parameters', 10)
        self.graphing_parameters_publisher = self.create_publisher(Float64MultiArray, '/graphing_parameters', 10)

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
