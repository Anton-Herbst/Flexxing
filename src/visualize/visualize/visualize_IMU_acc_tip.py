#!/usr/bin/env python3

"""
* This file is to see if the transformation and the following calculations are realistic.
* It will simply read bending and display the sensor readings as a robots pose.
"""

import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vector
from std_msgs.msg import Float64MultiArray                          # datatype to read custom bending topic
import matplotlib.pyplot as plt                                     # for visuals
import threading                                                    # so plt and ROS can work together
import numpy as np                                                  # useful for math stuff

class Visualizer_Tip(Node):
    def __init__(self):

        # "it insists upon itself"
        super().__init__('visualize_IMU_acc_tip')
        
        # * ROS related
        # create a subscriber to get the bending out of the nodes sensor/publish_tip released topics
        self.subscription_tip  =  self.create_subscription(
            msg_type    = Float64MultiArray, 
            topic       = '/pc/bending_IMU_acc_tip',
            callback    = self.callback_tip, 
            qos_profile = 10)
        # * Node parameters
        # robots length from bottom to tip (since we only use one sensor rn)
        self.length = 0.24 # keep this a float
        # * Visual related
        # interactive mode on
        plt.ion()  
        plt.style.use('_mpl-gallery')
        # create a window 
        self.fig = plt.figure(figsize = (8,8), num=f"Direction of the tip")
        # add an 3d plot in it
        self.axis = self.fig.add_subplot(projection='3d')
        self.axis.view_init(elev=30, azim=-65)
        # create a quiver (arrow/vector) starting upright
        self.vector = self.axis.quiver(0, 0, 0, 0, 0, self.length, color='r', arrow_length_ratio=0.1)
        self.axis.set_aspect('equal')
        # limit the axis
        self.axis.set_xlim([-self.length, self.length])
        self.axis.set_ylim([-self.length, self.length])
        # Robot cant go under the table so just look at the upper half
        self.axis.set_zlim([0, self.length])
        # label the axis
        self.axis.set_xlabel("X")
        self.axis.set_ylabel("Y")
        self.axis.set_zlabel("Z")
        # paint the coordinate system
        self.axis.quiver(0, 0, 0, self.length, 0, 0, color='k', arrow_length_ratio=0.1)
        self.axis.text(self.length, 0, 0, "X", color='k', fontsize=16)
        self.axis.quiver(0, 0, 0, 0, self.length, 0, color='k', arrow_length_ratio=0.1)
        self.axis.text(0, self.length, 0, "Y", color='k', fontsize=16)
        self.axis.quiver(0, 0, 0, 0, 0, self.length, color='k', arrow_length_ratio=0.1)
        self.axis.text(0, 0, self.length, "Z", color='k', fontsize=16)

    # * callback on receiving bending information
    def callback_tip(self, msg: Float64MultiArray) -> None: 
        # change the domain (lagrange to cartesion, with PCC thats forwards kinematics)
        endeffector = self.forward_kinematics(msg.data)
        # redraw the figure
        self.redraw(endeffector)
    
    # * function tasked to go from lagrangian coords to cartesian
    def forward_kinematics(self, data: list[float]) -> Vector3:
        # first extract the incoming messages data delta_x and delta_y
        [delta_x, delta_y, delta] = data
        # watch out for the singularity (no bending done)
        if delta < 1e-3:
            return Vector3(x=0.0, y=0.0, z=self.length)
        # prefactor for the vector (here singularity appears)
        factor = self.length / (delta ** 2)
        # prepare a vector
        position = Vector3()
        # fill it according to forward kinematics
        position.x = factor * (1 - np.cos(delta)) * delta_x
        position.y = factor * (1 - np.cos(delta)) * delta_y
        position.z = factor * np.sin(delta) * delta
        # return it
        return position

    # * function tasked to display new pose
    def redraw(self, new_vec: Vector3) -> None:
        # first delete the old quiver
        self.vector.remove()
        # then make a new one
        self.vector = self.axis.quiver(0,0,0, new_vec.x, new_vec.y, new_vec.z, color='r', arrow_length_ratio=0.1)
        # after all quivers have been created redraw the figure
        self.fig.canvas.draw()
        # dont let it freeze up
        self.fig.canvas.flush_events()

def main():
    rclpy.init()
    mynode = Visualizer_Tip()
    # spin the created note in a background tab 
    spin_thread = threading.Thread(target=rclpy.spin, args=(mynode,), daemon=True)
    spin_thread.start()
    # so that matplotlib can be in the foreground
    plt.show(block=True)
    # on exit clean up
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()