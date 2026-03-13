#! /usr/bin/env python3

"""
* This script takes the controllers output and applies it to the motors. Definition of a plant.
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⠀⠀
ITS A PLANT ⠀⠀⣠⡶⠛⠉⠉⢹⡆⠀
⠀⠀⠀⣀⠀⠀⠀⠀⠀⠀⠀⢰⡏⢀⡴⠀⠀⣾⠁⠀
⠰⣟⠋⠉⠙⠛⢶⣄⠀⠀⠀⢸⣷⠟⠁⣀⡼⠋⠀⠀
⠀⢻⡄⠀⠰⢦⣄⣹⡆⠀⠀⣼⠿⠛⠛⠉⠀⠀⠀⠀
⠀⠀⠻⢦⣄⣀⣈⣻⣿⡀⢰⡏⠀⠀⠀⠀⠀⣀⠀⠀
⠀⠀⠀⠀⠈⠉⠉⠉⠈⠻⣼⡇⠀⢀⡴⠛⠋⠉⢙⡿
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⡇⢀⣾⣥⠾⠃⢀⣼⠃
⠀⠀⠀⠀⣤⣤⣤⣤⣀⠀⢸⣇⣼⠿⠷⠶⠶⠛⠁⠀
⠀⠀⠀⠀⢻⡄⠀⣤⣹⣧⢸⡿⠁⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠛⢶⣤⣽⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⡇⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⡼⢧⣄⡀⠀⠀⠀⠀⠀
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from soro_msgs.msg import ServoCommands                             # datatype for publishing servos
from std_msgs.msg import Float64MultiArray                          # datatype used for multiple floats

class Plant(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('controller')

        # * ROS related
        # subscribe to controllers output (publised topics from /controller/PI_controller.py)
        self.subscription_top = self.create_subscription(Float64MultiArray, '/pc/control_output_top', lambda msg: self.callback_controller(msg, 'top'), 10)
        self.subscription_bot = self.create_subscription(Float64MultiArray, '/pc/control_output_bot', lambda msg: self.callback_controller(msg, 'top'),10)
        # link into  the publishing topic for the servo
        self.servo_pub = self.create_publisher(ServoCommands, '/teensy_hub/servo_pos', 10)
        
        # * variables of this node
        # order of the servo commands, important for publishing them later
        self.order = ['top1','bot1','top2','bot2','top3','bot3']
    
    # * callback function of the controller
    def callback_controller(self, msg: Float64MultiArray, segment: str):
        pass

    # * function to get the current motor position

def main():
    rclpy.init()
    mynode = Plant()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()