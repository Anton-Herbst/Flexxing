#! /usr/bin/env python3

"""
* This script takes the resulting tendon lengths from the G2L (from measurements) and X2L (from trajectory) inverse kinematics nodes.
* It then publishes the error between the real and the desired tendon lengths.
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from std_msgs.msg import Float64MultiArray                          # datatype used for multiple floats
from std_msgs.msg import Float64

class Publish_Tendon_Error(Node):
    
    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('publish_tendon_error')

        # * ROS related
        # subscribe to incoming tendon lengths for each segment published by kinematics/G2L_imu_acc_all, depending on sensor measurements
        self.subscription_real_bot = self.create_subscription(Float64MultiArray, '/pc/tendon_lengths_bot', lambda msg: self.tendon_lengths_real(msg, 'bot'), 10 )
        self.subscription_real_top = self.create_subscription(Float64MultiArray, '/pc/tendon_lengths_top', lambda msg: self.tendon_lengths_real(msg, 'top'), 10 )
        # subscribe to the desired tendon lengths published by kinematics/G2L_trajectory, depending on generation in controller package
        self.subscription_desired_bot = self.create_subscription(Float64MultiArray, '/pc/tendon_lengths_target_bot', lambda msg: self.tendon_lengths_desired(msg, 'bot'), 10 )
        self.subscription_desired_top = self.create_subscription(Float64MultiArray, '/pc/tendon_lengths_target_top', lambda msg: self.tendon_lengths_desired(msg, 'top'), 10 )
        # publisher giving out the error for all tendons for each segment
        self.publisher_error_bot = self.create_publisher(Float64MultiArray, '/pc/tendon_error_bot', 10)
        self.publisher_error_top = self.create_publisher(Float64MultiArray, '/pc/tendon_error_top', 10)
        # for later visuals its good to see the absolute error, as well as the average error
        self.publisher_absolute_error = self.create_publisher(Float64, '/pos/absolute_error', 10)
        self.publisher_average_error = self.create_publisher(Float64, '/pps/average_error', 10)

        # * Geometric Parameters
        self.segment_length = self.declare_parameter('L_segment', 0.12).value

        # * Parameters of this node
        # placeholder for all realistic and desired tendon lengths, to be able to calculate the error when both are available
        self.lengths_real = { 'bot': [self.segment_length] * 3, 'top': [self.segment_length] *3 }
        self.lengths_desired = { 'bot': [self.segment_length] * 3, 'top': [self.segment_length] *3 }

    # * callback on receiving new sensor info
    def tendon_lengths_real(self, msg: Float64MultiArray, segment: str) -> None:
        # extract incoming data
        self.lengths_real[segment] = msg.data
        # new data means new publishing
        self.calculate_and_publish_tendon_error()

    # * callback on receiving new desired tendon lengths
    def tendon_lengths_desired(self, msg: Float64MultiArray, segment: str) -> None:
        # extract incoming data
        self.lengths_desired[segment] = msg.data
        # new data means new publishing
        self.calculate_and_publish_tendon_error()

    # * function to publish the tendon error
    # since data comes in regulary we just do it all at once
    def calculate_and_publish_tendon_error(self) -> None:
        # calculate every error
        tendon_error_bot = np.asarray(self.lengths_desired['bot']) - np.asarray(self.lengths_real['bot'])
        tendon_error_top = np.asarray(self.lengths_desired['top']) - np.asarray(self.lengths_real['top'])
        # create the messages
        msg_bot, msg_top = Float64MultiArray(), Float64MultiArray()
        # populate them
        msg_bot.data, msg_top.data = tendon_error_bot, tendon_error_top
        # publish the messages
        self.publisher_error_bot.publish(msg_bot)
        self.publisher_error_top.publish(msg_top)
        # absolute error
        tendon_error_all = np.concatenate( (tendon_error_bot, tendon_error_top) )
        msg_abs = Float64()
        msg_abs.data = np.linalg.norm(tendon_error_all)
        self.publisher_absolute_error.publish(msg_abs)
        msg_avg = Float64()
        msg_avg.data = np.mean(tendon_error_all)
        self.publisher_average_error.publish(msg_avg)

def main():
    rclpy.init()
    mynode = Publish_Tendon_Error()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()