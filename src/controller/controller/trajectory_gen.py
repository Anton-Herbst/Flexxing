#!/usr/bin/env python3

"""
* This script generates the target trajectory that the robot will want to follow.
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from std_msgs.msg import Float64MultiArray                          # datatype used for publishing PCC values
# following libs are all needed to have parameters, makes tuning the controller easier
from rcl_interfaces.msg import ParameterDescriptor  

class Trajectore_gen(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('trajectory_gen')
        
        # * geometric paramter
        self.d = self.declare_parameter('d', 0.018).value

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
        self.trajectory_name = self.declare_parameter('trajectory_name', 'circle', ParameterDescriptor(description = 'String value telling the controller/trajectory_gen Node which trajectory to generate.') ).value
        self.get_logger().info(f'Selected trajectory is {self.trajectory_name}')
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
        if trajectory_name == 'circle':
            t = self.tick/self.hz
            # degrees around the z axis, moving in a circle
            phi = self.wrap_angle( 2 * np.pi * t/self.period_T )
            # defines amplitude of the circle on segment 1
            theta = np.deg2rad(15)
            delta_x1 = theta*self.d*np.cos(phi)
            delta_y1 = theta*self.d*np.sin(phi)
            delta_x2 = delta_x1
            delta_y2 = delta_y1
            return delta_x1, delta_y1, delta_x2, delta_y2
        elif trajectory_name == 'point_x':
            phi = np.deg2rad(0)
            theta = np.deg2rad(40)
            delta_x1 = 0
            delta_y1 = 0
            delta_x2 = theta*np.cos(phi)
            delta_y2 = theta*np.sin(phi)
            return delta_x1, delta_y1, delta_x2, delta_y2
        elif trajectory_name == 'point_xy':
            phi = np.deg2rad(45)
            theta = np.deg2rad(30)
            delta_x1 = 0
            delta_y1 = 0
            delta_x2 = theta*np.cos(phi)
            delta_y2 = theta*np.sin(phi)
            return delta_x1, delta_y1, delta_x2, delta_y2   
        elif trajectory_name == 'point_y':
            phi = np.deg2rad(270)
            theta = np.deg2rad(30)
            delta_x1 = 0
            delta_y1 = 0
            delta_x2 = theta*np.cos(phi)
            delta_y2 = theta*np.sin(phi)
            return delta_x1, delta_y1, delta_x2, delta_y2  
        elif trajectory_name == 'triangle':
            # normalize time that has passed to  to [0, 1)
            progress = ((self.tick/self.hz) / self.period_T ) % 1
            # triangle amplitude (same idea as circle)
            theta = np.deg2rad(30)
            A = theta * self.d
            # define triangle corners as x,y
            v1 = np.array([ A, 0])
            v2 = np.array([-A/2, -np.sqrt(3)/2 * A])
            v3 = np.array([-A/2, np.sqrt(3)/2 * A])
            # determine which segment we are in
            if progress < 1/3:
                # in just 1/3 of the progress time the entire border needs to be followed hence *3
                alpha = progress * 3
                # current position is starting a v1 with alpha=0 and ends at v2 at alpha=1
                p = (1 - alpha) * v1 + alpha * v2
            elif progress < 2/3:
                # again in just 1/3 the entire border needs to be run, here -1/3 needs to be aplied since 33% is done
                alpha = (progress - 1/3) * 3
                # current position is starting at v2 (where it ended before) and ends now at v4
                p = (1 - alpha) * v2 + alpha * v3
            else:
                # again in just 2/3 the entire border needs to be run, here -2/3 needs to be aplied since 66% is done
                alpha = (progress - 2/3) * 3
                # picks up at the last place v3 and will close the loop to v1
                p = (1 - alpha) * v3 + alpha * v1
            # since p is defined as the track between the corners it carriex x and y
            delta_x1, delta_y1 = p/2
            # because the robot has two segments its best to demonstrate the lower half too
            delta_x2, delta_y2 = p
            # this produces a nice triangle
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