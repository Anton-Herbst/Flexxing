#!/usr/bin/env python3

"""
* This file grabs the tips sensors transformed vector. From it the generalized coordinates are calulated and then published.
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
from std_msgs.msg import Float64MultiArray                          # datatype used for publishing PCC values

class Gen_coords_mag_tip(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('gen_coords_mag_tip')

        # * ROS related
        # subscribe to the transformed vector topic
        self.subscription = self.create_subscription(
            msg_type    = Vector3,
            topic       = '/pc/transformed_mag_tip',
            callback    = self.callback_mag_tip,
            qos_profile = 10)
        # create a publisher for the bending results
        self.publisher = self.create_publisher(
            msg_type    = Float64MultiArray,
            topic       = '/pc/gen_coords_mag_tip',
            qos_profile = 10)

    # * on receiving new info
    def callback_mag_tip(self, msg: Vector3) -> None:
        phi = self.get_bending_orientation(msg)
        theta = self.get_vertical_angle(msg)
        msg_pub = Float64MultiArray()
        msg_pub.data = list(self.get_generalized_coordinates(theta, phi))
        self.publisher.publish(msg_pub)

    # * function determing plane in which the robot is bent
    def get_bending_orientation(self, transformed: Vector3) -> float:
        phi = np.arctan2(transformed.y, transformed.x)
        return phi.item()

    # * function calculating angular offset to the rooms z-axis
    def get_vertical_angle(self, transformed: Vector3) -> float:
        planar_axis = np.sqrt(transformed.x ** 2 + transformed.y ** 2)
        theta = np.arctan2(planar_axis, transformed.z)
        return theta.item()

    # * function to get the generalized coordinates
    def get_generalized_coordinates(self, theta: float, phi: float) -> tuple[float, float, float]:
        delta_x = theta * np.cos(phi)
        delta_y = theta * np.sin(phi)
        return delta_x, delta_y, theta

def main():
    rclpy.init()
    mynode = Gen_coords_mag_tip()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()