#! /usr/bin/env python3

"""
* This script is the inverse kinematics of the PCC robot.
* Its purpose is to convert the desired trajectory to tendon lengths.
* endeffector X -> configuration G -> tendon lengths L
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
from std_msgs.msg import Float64MultiArray                          # datatype used for multiple floats

class Inverse_PCC_X2L(Node):
    
    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('inverse_PCC_X2L')

        # * ROS related
        # subscribe to incoming tendon lengths
        self.subscription = self.create_subscription(
            msg_type    =  Vector3,
            topic       = '/pc/controller/trajectory',
            callback    = self.callback_trajectory,
            qos_profile = 10 )
        # publisher giving away the tendon lengths # TODO : Make new node in package servos to receive this
        self.publisher_ = self.create_publisher( 
            msg_type    = Float64MultiArray,
            topic       = '/pc/servos/tendon_lengths',
            qos_profile = 10 )

    # * callback on receiving new tendon lengths
    def callback_trajectory(self, msg: Float64MultiArray) -> None:
        # read incoming target positions
        x, y, z = msg.x, msg.y, msg.z
        delta_x, delta_y, theta = self.inverse_kinematics_X2G(x, y, z)

    # * function describing inverse kinematics from endeffector X to configuration G
    def inverse_kinematics_X2G(self, x: float, y: float, z: float) -> np.ndarray:
        # TODO : implement this
        return np.array([0, 0, 0])
    
    # * function describing inverse kinematics from configuration G to tendon lengths L
    def inverse_kinematics_G2L(self, G: np.ndarray) -> np.ndarray:
        # TODO : implement this
        return np.array([0, 0, 0])

def main():
    rclpy.init()
    mynode = Inverse_PCC_X2L()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
