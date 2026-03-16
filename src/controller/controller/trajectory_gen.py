#!/usr/bin/env python3

"""
* This script generates the target trajectory that the robot will want to follow.
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
from std_msgs.msg import Float64MultiArray                          # datatype used for publishing PCC values
# following libs are all needed to have (dynamic) parameters, makes tuning the controller easier
from rcl_interfaces.msg import ParameterEvent
from rclpy.parameter_event_handler import ParameterEventHandler
from rcl_interfaces.msg import ParameterDescriptor  

class Trajectore_gen(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('gen_coords_imu_acc')

        # * ROS related
        # this will publish (x, y, z, theta) of the tip
        self.trajectory_publisher = self.create_publisher(Float64MultiArray, '/pc/trajectory_coords', 10)
        # frequency of the updates
        hz = 60
        # timer dictating how often publishing topic gets pushed
        self.publishing_timer = self.create_timer( 1/hz, self.publishing_timer_callback)
        # save the starting time
        self.starting_time = self.publishing_timer.clock.now().nanoseconds
        # this node gets launched with a parameter deciding which trajectory gets followed
        self.trajectory_name = self.declare_parameter('Name of the Trajecoty', 'circle_xy', ParameterDescriptor = 'String value telling the controller/trajectory_gen Node which trajectory to generate.')
        
        # * Parameter for trajectory generation
        # time in seconds for a completed trajectory
        self.period_T = 10

    # * callback of the timer
    def publishing_timer_callback(self):
        # get time that has been 
        t = ( self.publishing_timer.clock.now().nanoseconds - self.starting_time ) * 1e-9
        # get the target coordinates
        x, y, z, theta = self.function_generation(t, self.trajectory_name)
        # publish the trajectory
        self.publish_trajectory(x, y, z, theta)
        pass

    # * function generator selected by parameter
    def function_generation(self, t: int, trajectory_name: str) -> tuple[float, float, float, float]:
        # create variable space
        x, y, z, theta = 0.0, 0.0, 0.0, 0.0
        if trajectory_name == 'circle_xy':
            # degrees around the z axis, moving in a circle
            phi = self.wrap_angle( 2 * np.pi * t/self.period_T )
            # radius in m
            radius = 0.2
            # planar circular motion
            x = radius * np.cos(phi)
            y = radius * np.sin(phi)
            # z is predfined
            z = 0.2
            # degrees from the z axis, predefined
            theta = 10
        return x, y, z, theta
    
    # * function to keep angles in the interval [0, 2*pi)
    def wrap_angle(self, angle = float) -> float:
        return float( angle % (2 * np.pi) )
    
    # * function to publish
    def publish_trajectory(self, x:float, y:float, z: float, theta: float) -> None:
        # create a message with fitting type
        msg = Float64MultiArray()
        # populate with relevant data
        msg.data = [x, y, z, theta]
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