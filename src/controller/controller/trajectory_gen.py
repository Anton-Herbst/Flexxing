#!/usr/bin/env python3

"""
* This script generates the target trajectory that the robot will want to follow.
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from std_msgs.msg import Float64MultiArray                          # datatype used for publishing PCC values
# following libs are all needed to have (dynamic) parameters, makes tuning the controller easier
from rcl_interfaces.msg import ParameterEvent
from rclpy.parameter_event_handler import ParameterEventHandler
from rcl_interfaces.msg import ParameterDescriptor  

class Trajectore_gen(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('trajectory_gen')
        
        # * ROS related
        # this will publish (delta_x1, delta_y1, delta_x2, delta_y2), full configuration of the robot
        self.trajectory_publisher = self.create_publisher(Float64MultiArray, '/pc/controller/trajectory', 10)
        # frequency of the updates
        self.hz = 100
        # ticker of how long its running
        self.tick = 0
        # timer dictating how often publishing topic gets pushed
        self.publishing_timer = self.create_timer( 1/self.hz, self.publishing_timer_callback)
        # this node gets launched with a parameter deciding which trajectory gets followed
        self.trajectory_name = self.declare_parameter('Name of the Trajecoty', 'circle_xy', ParameterDescriptor(description = 'String value telling the controller/trajectory_gen Node which trajectory to generate.') ).value
        
        # * Parameter for trajectory generation
        # time in seconds for a completed trajectory
        self.period_T = 10

    # * callback of the timer
    def publishing_timer_callback(self) -> None:
        # on callback uodate the tick
        self.tick += 1
        # get the target coordinates
        delta_x1, delta_y1, delta_x2, delta_y2 = self.function_generation(self.trajectory_name)
        # publish the trajectory
        self.fire_away(delta_x1, delta_y1, delta_x2, delta_y2 )

    # * function generator selected by parameter
    # it returns generalized coordinates for bottom and top segment
    def function_generation(self, trajectory_name: str) -> tuple[float, float, float, float]:
        t = self.tick/self.hz
        if trajectory_name == 'circle_xy':
            # degrees around the z axis, moving in a circle
            phi = self.wrap_angle( 2 * np.pi * t/self.period_T )
            # defines amplitude of the circle on segment 1
            theta = np.deg2rad(20)
            delta_x1 = theta*np.cos(phi)
            delta_x2 = delta_x1
            delta_y1 = theta*np.sin(phi)
            delta_y2 = delta_y1
            return delta_x1, delta_y1, delta_x2, delta_y2
        else:
            return 0.0, 0.0, 0.0, 0.0
        
    # * function to keep angles in the interval [0, 2*pi)
    def wrap_angle(self, angle: float) -> float:
        return float( angle % (2 * np.pi) )   
    
    # * function that will publish the generated values
    def fire_away(self, delta_x1: float, delta_y1: float, delta_x2: float, delta_y2: float) -> None:
        # prepare a message
        msg = Float64MultiArray()
        # populate it
        msg.data = [ delta_x1, delta_y1, delta_x2, delta_y2 ]
        # fire it off
        self.trajectory_publisher.publish(msg)

def main():
    rclpy.init()
    mynode = Trajectore_gen()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()