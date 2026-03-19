#! /usr/bin/env python3

"""
* This script is partial inverse kinematics of the PCC robot.
* It takes the generealized coordinates produced by controller/trajectory_gen and gives out tendon lengths
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from std_msgs.msg import Float64MultiArray                          # datatype used for multiple floats

class Inverse_PCC_G2L_traj(Node):
    
    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('inverse_PCC_G2L_trajectory')

        # * ROS related
        # subscribe to incoming generalized coordinates produced by controller/trajectory to get all generealiezd coordinates
        self.subscription_bot = self.create_subscription(Float64MultiArray, '/pc/controller/trajectory', self.callback_trajectory, 10 )
        # publisher giving out lengths for all tendons for each segment
        self.publisher_top = self.create_publisher( Float64MultiArray, '/pc/tendon_lengths_target_top', 10 )
        self.publisher_bot = self.create_publisher( Float64MultiArray, '/pc/tendon_lengths_target_bot', 10 )
        # create a publisher dict to select where to publish the results 
        self.publisher_select = { 'bot': self.publisher_bot, 'top': self.publisher_top }

        # * Geometric Parameters
        # the length of the segment (since we only look at the upper point)
        self.segment_length = self.declare_parameter('L_segment', 0.12).value
        # distance to the middle arc
        self.d = self.declare_parameter('d', 0.018).value
        # topper rotation
        self.yaw_offset = np.deg2rad(300)

    # * callback on receiving new info
    def callback_trajectory(self, msg: Float64MultiArray) -> None:
        # extract incoming data
        delta_x_bot, delta_y_bot, delta_x_top, delta_y_top = msg.data
        # calculate the tendon lengths
        l_1, l_2, l_3 = self.inverse_kinematics_G2L(delta_x_bot, delta_y_bot, 'bot')
        # publish the tendon lengths
        self.publish_tendon_lengths(l_1, l_2, l_3, 'bot')
        # calculate the tendon lengths
        l_4, l_5, l_6 = self.inverse_kinematics_G2L(delta_x_top, delta_y_top, 'top')
        # publish the tendon lengths
        self.publish_tendon_lengths(l_4, l_5, l_6, 'top')

    # * function to calculate tendon lengths from the generalized coordinates
    def inverse_kinematics_G2L(self, delta_x: float, delta_y: float, segment: str) -> tuple[float, float, float]:
        # prepare variables in this scope to save the tendon lengths
        l_a, l_b, l_c = 0.0, 0.0, 0.0
        if segment == 'top': # -> l4, l5, l6
            # the top segment tendons are rotated by 60° 
            l_a = self.segment_length + self.d * (delta_x * np.cos(self.yaw_offset)           + delta_y * np.sin(self.yaw_offset))
            l_b = self.segment_length + self.d * (delta_x * np.cos(self.yaw_offset+2/3*np.pi) + delta_y * np.sin(self.yaw_offset+2/3*np.pi))
            l_c = self.segment_length + self.d * (delta_x * np.cos(self.yaw_offset+4/3*np.pi) + delta_y * np.sin(self.yaw_offset+4/3*np.pi))
        elif segment == 'bot': # -> l1, l2, l3
            # the bottom segment tendons are not rotated
            l_a = self.segment_length + self.d * (delta_x * np.cos(0)         + delta_y * np.sin(0))
            l_b = self.segment_length + self.d * (delta_x * np.cos(2/3*np.pi) + delta_y * np.sin(2/3*np.pi))
            l_c = self.segment_length + self.d * (delta_x * np.cos(4/3*np.pi) + delta_y * np.sin(4/3*np.pi))
        return l_a, l_b, l_c    
    
    # * function pushing data to publisher
    def publish_tendon_lengths(self, l_a: float, l_b: float, l_c: float, segment: str) -> None:
        # prepare a message
        msg = Float64MultiArray()
        # load the tendon lengths into the message
        msg.data = [l_a, l_b, l_c]
        # publish the message
        self.publisher_select[segment].publish(msg)


def main():
    rclpy.init()
    mynode = Inverse_PCC_G2L_traj()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
