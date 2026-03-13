#! /usr/bin/env python3

"""
* This file is to see if the transformation and the following calculations are realistic.
* It will read in the position of the endeffector directly from the kinematics package and just display it.
"""

import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vector
from std_msgs.msg import Float64MultiArray                          # datatype to read custom bending topic
import matplotlib.pyplot as plt                                     # for visuals
import threading                                                    # so plt and ROS can work together
import numpy as np                                                  # useful for math stuff

class Visualizer_imu_acc_tip(Node):

    # * function called on creation
    def __init__(self):

        # "it insists upon itself"
        super().__init__('visualize_imu_acc_tip')
        
        # * ROS related
        # create a subscriber to get the endeffectors position from kinematics/G2X_imu_acc_tip
        self.subscription_tip  =  self.create_subscription( Vector3, '/pos/endeffector', self.callback_tip, 10 )
        # * Node parameters
        # robots length from bottom to tip (since we only use one sensor rn)
        self.length = 0.24 
        # * Visual related
        # interactive mode on
        plt.ion()  
        plt.style.use('_mpl-gallery')
        # create a window 
        self.fig = plt.figure(figsize = (8,8), num=f"Direction of the tip, using imu linear acceleration data")
        # add an 3d plot in it
        self.axis = self.fig.add_subplot(projection='3d')
        self.axis.view_init(elev=30, azim=20)
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
    # redraw the figure with the new pose
    def callback_tip(self, msg: Vector3) -> None: self.redraw(msg)
        
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
    mynode = Visualizer_imu_acc_tip()
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