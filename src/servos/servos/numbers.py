#!/usr/bin/env python3

"""
* This file is for me to figure out which servo has which number.
------------------------------------------------------------------
0,2,4 - bottom half
1,3,5 - higher half
"""

import numpy as np                          # math stuff useful for making the servos do curves
import rclpy                                # to be able to use ROS with python
from rclpy.node import Node                 # ROS node creation
from soro_msgs.msg import ServoCommands     # datatype for publishing servos

class Numbering(Node):
    # * function called on creation
    def __init__(self):
        # "it insists upon itself"
        super().__init__('servo_numbering')
        # * Parameters
        # running variable which servo is targeted
        self.number = 1
        self.get_logger().info(f'Wiggling Motor #{self.number}')
        # period of our sinus (in seconds)
        self.T = 1
        # resolution of the timer/how often servos will be updated during period
        self.res = 1000
        # Amplitude of our sinus (ranged from min 0 to max 15000)
        self.amp = 1000
        # check flag to end the process over
        self.done = False
        # * ROS topics
        # link into the servo topic to command the robots actuators
        self.servo_pub = self.create_publisher(
            msg_type    = ServoCommands,
            topic       = '/teensy_hub/servo_pos',
            qos_profile = 10)
        # create a timer to oversee progress
        self.timer = self.create_timer(
            timer_period_sec    = self.T/self.res,
            callback            = self.timer_callback)
        # mark down the starting point of the timer
        self.start_time = self.timer.clock.now().nanoseconds
    # * callback function of the timer
    def timer_callback(self):
        # calculate time (in s) since the timers creation
        t = (self.timer.clock.now().nanoseconds - self.start_time)/10**9
        # if time is less then one period
        if t < self.T * self.number:
            # neutral positions for the servos 
            custom = [1500]*12
            # make one servo wiggle to identify its number IRL (arrays start at 0)
            custom[self.number-1] = self.amp * np.sin(2 * np.pi / self.T * t) + 15000
            # create a msg type for the servos
            servo_msg = ServoCommands()
            # fill the msg with the created custom values
            servo_msg.servo_micros = custom
            # publish the full msg to the targeted servo
            self.servo_pub.publish(servo_msg)
        # if one period is over 
        else:
            # advance to the next servo 
            self.number += 1
            # only the first 6 servos of the robots platine are used, so the rest is whatever
            if self.number > 6:
                self.get_logger().info(f"Done wiggling")
                self.done = True
                return
            else:
                self.get_logger().info(f'Wiggling Motor #{self.number}')
def main():
    rclpy.init()
    node = Numbering()
    while not node.done:
        rclpy.spin_once(node)
    # Cleanup on shutdown
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()