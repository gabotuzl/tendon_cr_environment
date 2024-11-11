#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from moviepy.editor import VideoClip
from moviepy.video.io.bindings import mplfig_to_npimage

import time


plt.switch_backend('TkAgg')
plt.close("all")

class Visualizer(Node):

    def __init__(self):
        super().__init__("visualizer_node")
        self.get_logger().info("Visualizer node has been started.")

        # Creating subscribers
        self.graphing_parameters_subscriber = self.create_subscription(Float64MultiArray, "/graphing_parameters", self.graphing_parameters_callback,10)
        self.position_history_subscriber = self.create_subscription(Float64MultiArray, "/position_history", self.position_history_callback, 10)
        self.director_history_subscriber = self.create_subscription(Float64MultiArray, "/director_history", self.director_history_callback, 10)

    def make_frame(self,t):
        # Method to get the frames

        # Clear
        self.ax.clear()
        
        # Scatter plot
        self.ax.scatter(self.rod_history[self.count][0], self.rod_history[self.count][1], self.rod_history[self.count][2])
        self.ax.axes.set_zlim3d(bottom = self.zmin,top = self.zmax)
        self.ax.axes.set_ylim3d(bottom = self.ymin,top = self.ymax)
        self.ax.axes.set_xlim(-0.05,self.base_length)

        # Labeling axes
        self.ax.set_xlabel('X-axis')
        self.ax.set_ylabel('Y-axis')
        self.ax.set_zlabel('Z-axis')

        # Annotation for the tip position
        x_point = round(self.rod_history[self.count][0][-1],3)
        y_point = round(self.rod_history[self.count][1][-1],3)
        z_point = round(self.rod_history[self.count][2][-1],3)
        annotation_text = f'Tip Pos: ({x_point}, {y_point}, {z_point})'
        self.ax.text(x_point, y_point, z_point, annotation_text, color='red')

        # Adds local reference frames to the vertebra nodes
        for node in self.vertebra_nodes:
            local_directors = self.director_history[self.count][...,node]
            vertebra_pos = np.array([self.rod_history[self.count][0][node], self.rod_history[self.count][1][node], self.rod_history[self.count][2][node]])
            vertebra_coord_syst_x = np.array([local_directors[0][0], local_directors[0][1], local_directors[0][2]])
            vertebra_coord_syst_y = np.array([local_directors[1][0], local_directors[1][1], local_directors[1][2]])
            vertebra_coord_syst_z = np.array([local_directors[2][0], local_directors[2][1], local_directors[2][2]])

            # Scales the arrows for better visibility
            scale = 0.04

            # Plots the arrows using quiver
            self.ax.quiver(vertebra_pos[0], vertebra_pos[1], vertebra_pos[2],
                vertebra_coord_syst_x[0], vertebra_coord_syst_x[1], vertebra_coord_syst_x[2],
                length=scale, color='r', normalize=True)
            self.ax.quiver(vertebra_pos[0], vertebra_pos[1], vertebra_pos[2],
                vertebra_coord_syst_y[0], vertebra_coord_syst_y[1], vertebra_coord_syst_y[2],
                length=scale, color='g', normalize=True)
            self.ax.quiver(vertebra_pos[0], vertebra_pos[1], vertebra_pos[2],
                vertebra_coord_syst_z[0], vertebra_coord_syst_z[1], vertebra_coord_syst_z[2],
                length=scale, color='b', normalize=True)

        # Update counter
        self.count=self.count+1
        
        # Returning numpy image
        return mplfig_to_npimage(self.fig)
    
    def graphing_parameters_callback(self, msg:Float64MultiArray):
        # Receives parameters used for the visualization of the simulation and saves them as attributes
        self.graphing_parameters = msg.data

        # Set graphing parameters
        self.base_length = msg.data[0]
        self.array_size = int(msg.data[1]+1) # Calculates array size for X, Y, and Z in rod_history (msg.data[1] is n_elements)
        self.final_time = msg.data[2] 
        self.rendering_fps = msg.data[3]       
        self.zmin = -self.base_length
        self.zmax = self.base_length
        self.ymin = -self.base_length
        self.ymax = self.base_length
        self.num_frames = int(self.final_time * self.rendering_fps + 1) #Calculates total amount of frames in video 
        self.num_arrays = int(3 * self.num_frames) #Calculates total amount of X, Y and Z arrays in rod_history 

        num_vertebrae = int(msg.data[4])
        first_vertebra_node = int(msg.data[5])
        final_vertebra_node = int(msg.data[6])
        self.vertebra_height = msg.data[7]
        vertebra_increment = (final_vertebra_node - first_vertebra_node)/(num_vertebrae - 1)
        self.vertebra_nodes = []

        # Calculates which nodes in the rod have vertebrae
        for i in range(num_vertebrae):
            self.vertebra_nodes.append(round(i * vertebra_increment + first_vertebra_node))

    def position_history_callback(self, msg:Float64MultiArray):
        # Receives the position history of the rod and saves it as an attribute, later starting the video creation process

        # Unflattening the msg.data received for position_history
        self.rod_history = np.reshape(msg.data, (int(self.final_time * self.rendering_fps + 1), 3, int(self.array_size)))

        # Creating a 3D plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        # Initialize counter
        self.count=0

        # Creating animation
        clip = VideoClip(self.make_frame, duration = self.final_time)
        
        # Displaying animation with auto play and looping
        clip.write_videofile("Rod_Simulation.mp4", codec = "libx264", fps = self.rendering_fps)

        # This method makes the final frame of the video, and it allows the user to look at the 3D plot more clearly
        self.make_final_frame()
    
    def director_history_callback(self, msg:Float64MultiArray):
        # Receives the history of the local directors of the rod, which are used to create the local reference frames in the visualization

        # Unflattening the msg.data received for director_history
        self.director_history = np.reshape(msg.data, (int(self.final_time * self.rendering_fps + 1), 3, 3, int(self.array_size)-1))
    
    def make_final_frame(self):
        # This method makes the final frame of the video, and it allows the user to look at the 3D plot more clearly

        plt.close('all')
        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.set_title('Final pose of the continuum robot', fontsize= 22)
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')

        # Scatter plot
        ax.scatter(self.rod_history[-1][0], self.rod_history[-1][1], self.rod_history[-1][2])
        ax.axes.set_zlim3d(bottom = self.zmin,top = self.zmax)
        ax.axes.set_ylim3d(bottom = self.ymin,top = self.ymax)
        ax.axes.set_xlim(-0.05,self.base_length)

        # Labeling axes
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')

        calculated_x = self.rod_history[-1][0][-1]
        calculated_y = self.rod_history[-1][1][-1]
        calculated_z = self.rod_history[-1][2][-1]

        x_point = round(calculated_x,3)
        y_point = round(calculated_y,3)
        z_point = round(calculated_z,3)
        annotation_text = f'Tip Pos: ({x_point}, {y_point}, {z_point})'
        ax.text(x_point, y_point, z_point, annotation_text, color='red', fontsize= 15)

        # Adds local reference frames to the vertebra nodes
        for node in self.vertebra_nodes:
            local_directors = self.director_history[-1][...,node]
            vertebra_pos = np.array([self.rod_history[-1][0][node], self.rod_history[-1][1][node], self.rod_history[-1][2][node]])
            vertebra_coord_syst_x = np.array([local_directors[0][0], local_directors[0][1], local_directors[0][2]])
            vertebra_coord_syst_y = np.array([local_directors[1][0], local_directors[1][1], local_directors[1][2]])
            vertebra_coord_syst_z = np.array([local_directors[2][0], local_directors[2][1], local_directors[2][2]])

            # Scales the arrows for better visibility
            scale = 0.04

            # Plots the arrows using quiver
            ax.quiver(vertebra_pos[0], vertebra_pos[1], vertebra_pos[2],
                vertebra_coord_syst_x[0], vertebra_coord_syst_x[1], vertebra_coord_syst_x[2],
                length=scale, color='r', normalize=True)
            ax.quiver(vertebra_pos[0], vertebra_pos[1], vertebra_pos[2],
                vertebra_coord_syst_y[0], vertebra_coord_syst_y[1], vertebra_coord_syst_y[2],
                length=scale, color='g', normalize=True)
            ax.quiver(vertebra_pos[0], vertebra_pos[1], vertebra_pos[2],
                vertebra_coord_syst_z[0], vertebra_coord_syst_z[1], vertebra_coord_syst_z[2],
                length=scale, color='b', normalize=True)

        ax.view_init(elev=30, azim=-20)
        plt.show()

    

def main(args=None):
    rclpy.init(args=args)
    visualizer_node=Visualizer()
    try:
        rclpy.spin(visualizer_node)
    except KeyboardInterrupt:
        pass
    visualizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()