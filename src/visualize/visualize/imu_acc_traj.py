#!/usr/bin/env python3

"""
* This file is to see if the transformation and the following calculations are realistic.
* It will read in the position of the endeffector and the trajectory and display both.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
import threading

class Visualizer_imu_acc_tip(Node):

    # * function called on creation
    def __init__(self):
        # "it insists upon itself"
        super().__init__('visualize_imu_acc_tip')

        # * ROS related
        # create a subscriber to get the endeffectors position from kinematics/G2X_imu_acc_all
        self.subscription_tip       = self.create_subscription(Vector3, '/pos/endeffector',  self.callback_tip,  10)
        # create a subscriber to get the trajectory position from kinematics/G2X_trajectory
        self.subscription_trajectory = self.create_subscription(Vector3, '/pos/trajectory', self.callback_traj, 10)

        # * Node parameters
        self.length = 0.24

        # * Visual related
        plt.ion()
        plt.style.use('_mpl-gallery')
        self.fig = plt.figure(figsize=(8,8), num='Endeffector vs Trajectory')
        self.axis = self.fig.add_subplot(projection='3d')
        self.axis.view_init(elev=30, azim=20)
        # red quiver for endeffector
        self.vector      = self.axis.quiver(0, 0, 0, 0, 0, self.length, color='r', arrow_length_ratio=0.1)
        # blue quiver for trajectory
        self.vector_traj = self.axis.quiver(0, 0, 0, 0, 0, self.length, color='b', arrow_length_ratio=0.1)
        self.axis.set_aspect('equal')
        self.axis.set_xlim([-self.length, self.length])
        self.axis.set_ylim([-self.length, self.length])
        self.axis.set_zlim([0, self.length])
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
        # add legend
        self.axis.legend(
            handles=[
                plt.Line2D([0],[0], color='r', label='Endeffector'),
                plt.Line2D([0],[0], color='b', label='Trajectory')],
            loc='upper left')

    # * callbacks
    def callback_tip(self, msg: Vector3) -> None:
        self.vector.remove()
        self.vector = self.axis.quiver(0, 0, 0, msg.x, msg.y, msg.z, color='r', arrow_length_ratio=0.1)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def callback_traj(self, msg: Vector3) -> None:
        self.vector_traj.remove()
        self.vector_traj = self.axis.quiver(0, 0, 0, msg.x, msg.y, msg.z, color='b', arrow_length_ratio=0.1)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main():
    rclpy.init()
    mynode = Visualizer_imu_acc_tip()
    spin_thread = threading.Thread(target=rclpy.spin, args=(mynode,), daemon=True)
    spin_thread.start()
    plt.show(block=True)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()