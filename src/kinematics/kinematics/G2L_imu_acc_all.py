#! /usr/bin/env python3

"""
* This script is partial inverse kinematics of the PCC robot.
* Its purpose is to convert incoming generalized coordinates to tendon lengths.
* configuration G -> tendon length L
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
from std_msgs.msg import Float64MultiArray                          # datatype used for multiple floats

class Inverse_PCC_G2L(Node):
    
    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('inverse_PCC_G2L')

        # * ROS related
        # subscribe to incoming generalized coordinates for each segment
        self.subscription_bot = self.create_subscription(
            msg_type    =  Float64MultiArray,
            topic       = '/pc/gen_coords_imu_acc_bot',
            callback    = lambda msg: self.gen_coords_imu_acc(msg, 'bot'),
            qos_profile = 10 )
        self.subscription_top = self.create_subscription(
            msg_type    =  Float64MultiArray,
            topic       = '/pc/gen_coords_imu_acc_top',
            callback    = lambda msg: self.gen_coords_imu_acc(msg, 'top'),
            qos_profile = 10 )
        # publisher giving out current real position
        self.publisher_select = {
            'transform': {
                'bot': self.create_publisher( Vector3, '/pos/transform_bot', 10 ),
                'top': self.create_publisher( Vector3, '/pos/transform_top', 10 ),},
            'rotate': {
                'bot': self.create_publisher( Float64MultiArray, '/pos/rotate_bot', 10 ),
                'top': self.create_publisher( Float64MultiArray, '/pos/rotate_top', 10 ),}}
        # * Parameter
        self.segment_length = self.declare_parameter('L_segment', 0.12).value

def main():
    rclpy.init()
    mynode = Inverse_PCC_G2L()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
