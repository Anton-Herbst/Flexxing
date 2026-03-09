#!/usr/bin/env python3

"""
* This file grabs the sensor's transformed vector. From it deltax and deltay are derived and published.

TODO list
! change publish_tip to publish transformed Vectors. /pc/transformed_tip
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
from std_msgs.msg import Float64MultiArray                          # datatype used for publishing PCC values

class Bending_IMU_Acc(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('bending_IMU_acc')
        # * ROS related
        # subscribe to the transformed vector topic
        self.subscription = self.create_subscription(
            msg_type    = Vector3,
            topic       = '/pc/transformed_IMU_acc_tip',
            callback    = self.callback_tip,
            qos_profile = 10 )
        # create a publisher for the bending results
        self.publisher = self.create_publisher(
            msg_type    = Float64MultiArray,            # it will transport [deltax, deltay, delta (the norm of the other two)]
            topic       = '/pc/bending_IMU_acc_tip',
            qos_profile = 10 )
    
    # * on receiving new info
    def callback_tip(self, msg) -> None:
        # use the other functions and then publish
        phi = self.get_bending_orientation(msg)
        theta = self.get_vertical_angle(msg)
        msg_pub = Float64MultiArray()
        msg_pub.data = list(self.get_bending(phi, theta))
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
    
    # * function giving bending [delta_x, delty_y, delta]
    def get_bending(self, phi: float, theta: float) -> tuple[float, float, float]:
        delta_x = theta*np.cos(phi)
        delta_y = theta*np.sin(phi)
        # delta = theta (no need to compute that, but a*cos²+*a*sin² = a)
        return (delta_x.item(), delta_y.item(), theta)
    
def main():
    rclpy.init()
    mynode = Bending_IMU_Acc()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()