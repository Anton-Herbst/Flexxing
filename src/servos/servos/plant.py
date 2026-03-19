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
        super().__init__('plant')
        # * Parameter deciding wether to use open or closed loop control
        # control mode: 'open_loop' or 'closed_loop'
        self.control_mode    = self.declare_parameter('control_mode', 'open_loop').value

        # * ROS related
        # select topics based on control mode
        if self.control_mode == 'closed_loop':
            subscription_topic_top = '/pc/controller/output_top'
            subscription_topic_bot = '/pc/controller/output_bot'
        else:  # open_loop
            subscription_topic_top = '/pc/tendon_lengths_target_top'
            subscription_topic_bot = '/pc/tendon_lengths_target_bot'
        self.get_logger().info(f'Plant running in {self.control_mode} mode.')

        # * ROS related
        # subscribe to controllers output (publised topics from /controller/PI_controller.py)
        self.subscription_top = self.create_subscription(Float64MultiArray, subscription_topic_top, lambda msg: self.callback_controller(msg, 'top'), 10)
        self.subscription_bot = self.create_subscription(Float64MultiArray, subscription_topic_bot, lambda msg: self.callback_controller(msg, 'bot'),10)
        # link into  the publishing topic for the servo
        self.servo_pub = self.create_publisher(ServoCommands, '/teensy_hub/servo_pos', 10)
        
        # * geometric parameters
        # diameter of wheel is 4.8cm ~ 0.024m radius
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.024).value
        self.circumferance = 2*np.pi*self.wheel_radius
        self.segment_length = self.declare_parameter('L_segment', 0.12).value
        self.max_delta = 50/360 * 2 * np.pi * self.wheel_radius
        
        # * variables of this node
        # keep track of the tuned lengths, start at neutral lengths
        self.current_length = {'top': np.full(3, self.segment_length), 'bot': np.full(3, self.segment_length)}
        # order of the servo commands, important for publishing them later
        self.order = ['top1','bot1','top2','bot2','top3','bot3']
    
    # * callback function of the controller
    def callback_controller(self, msg: Float64MultiArray, segment: str) -> None:
        # read the incoming control output (k_p * error + k_i * integral)
        target_length = np.asarray(msg.data)
        # apply the control output directly to the current length
        self.current_length[segment] = target_length
        # now control servos
        self.activate_servos()

    # * function to give motors a new input
    def activate_servos(self) -> None:
        # prepare a message
        msg = ServoCommands()
        # prepare the microseconds 
        micros = [1500] * 12
        # enter the saved values from current lengths
        for idx in range(3):
            # values in micros[0]=top1, micros[2]=top2, micros[4]=top3
            micros[idx * 2]     = self.length_to_micros(self.current_length['top'][idx])
            # values in micros[1]=bot1, micros[3]=bot2, micros[5]=bot3
            micros[idx * 2 + 1] = self.length_to_micros(self.current_length['bot'][idx])
        # populate the message
        msg.servo_micros = micros
        # fire it off
        self.servo_pub.publish(msg)

    # * function to convert lengths to micros
    def length_to_micros(self, length: float) -> int:
        # difference to neutral position
        delta = length - self.segment_length
        # get how much has to be spooled off the roll
        rotations = delta / self.circumferance
        # convert that into degrees
        degrees = rotations * 360
        # map the calculated degrees over the range -50° to 50° responding to 900us to 2100us
        micros = int( np.interp(degrees, [-50, 50], [900, 2100]) )
        # give the result away
        return micros
    
def main():
    rclpy.init()
    mynode = Plant()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()