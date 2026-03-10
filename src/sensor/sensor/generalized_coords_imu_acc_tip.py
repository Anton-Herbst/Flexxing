#!/usr/bin/env python3

"""
* This file grabs the tips sensors transformed vector. From it the generalized coordinates are calulated and then published.
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
from std_msgs.msg import Float64MultiArray                          # datatype used for publishing PCC values

class Gen_coords_imu_acc_tip(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('gen_coords_imu_acc_tip')

        # * ROS related
        # subscribe to the transformed vector topic
        self.subscription = self.create_subscription(
            msg_type    = Vector3,
            topic       = '/pc/transformed_imu_acc_tip',
            callback    = self.callback_acc_tip,
            qos_profile = 10 )
        # create a publisher for the bending results
        self.publisher = self.create_publisher(
            msg_type    = Float64MultiArray,            # it will transport [delta_x, delta_y, delta (the norm)] for the tip sensor
            topic       = '/pc/gen_coords_imu_acc_tip',
            qos_profile = 10 )
    
    # * on receiving new info
    def callback_acc_tip(self, msg: Vector3) -> None:
        # use the other functions and then publish
        phi = self.get_bending_orientation(msg)
        theta = self.get_vertical_angle(msg)
        msg_pub = Float64MultiArray()
        msg_pub.data = list(self.get_generalized_coordinates(theta, phi))
        self.publisher.publish(msg_pub)

    # * function determing plane in which the robot is bent
    def get_bending_orientation(self, transformed: Vector3) -> float:
        # plane is described by rotation around z
        phi = np.arctan2(transformed.y, transformed.x)
        # transform back to python float and return it
        return phi.item()
    
    # * function calculating angular offset to the rooms z-axis
    def get_vertical_angle(self, transformed:Vector3) -> float:
        # the curvate is defined as k= 1/r = °/l and since we are interested in delta_x and delta_y l will be crossed out
        planar_axis = np.sqrt(transformed.x ** 2 + transformed.y ** 2)
        theta = np.arctan2(planar_axis, transformed.z)
        # return a python float
        return theta.item()
    
    # * function to get the generalized coordinates
    def get_generalized_coordinates(self, theta:float, phi: float) -> tuple[float, float, float]:
        # this only works with local angles, which we calculate beforehand
        delta_x = theta * np.cos(phi)
        delta_y = theta * np.sin(phi)
        return delta_x, delta_y, theta

def main():
    rclpy.init()
    mynode = Gen_coords_imu_acc_tip()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()