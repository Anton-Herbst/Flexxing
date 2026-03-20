#! /usr/bin/env python3

"""
* This script is partial forward kinematics of the PCC robot.
* Its purpose is to convert incoming generalized coordinates for only the tip to the endeffector position.
* Under PCC this is equivalent to only using one segment.
* configuration G -> endeffector X
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
from std_msgs.msg import Float64MultiArray                          # datatype used for multiple floats

class G2X_mag_tip(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('forward_PCC_G2X_mag_tip')

        # * ROS related
        # subscribe to incoming generalized coordinates for the tip from sensor/gen_coords_mag_tip
        self.subscription = self.create_subscription(Float64MultiArray, '/pc/gen_coords_mag_tip', self.callback_gen_coords_tip, 10)
        # publisher giving out current real position according to forward kinematics
        self.publisher_endeffector = self.create_publisher(Vector3, '/pos/endeffector', 10)

        # * Parameter
        # only one segment means total length of the robot
        self.segment_length = self.declare_parameter('L_total', 0.24).value

    # * callback on receiving new generalized coordinates
    def callback_gen_coords_tip(self, msg: Float64MultiArray) -> None:
        # read incoming configuration
        delta_x, delta_y, delta = msg.data
        # calculate transformation vector
        transformation_vector = self.get_transformation_vector(delta_x, delta_y, delta)
        # fire it away
        self.publisher_endeffector.publish(transformation_vector)

    def get_rotation_matrix(self, delta_x: float, delta_y: float, delta: float) -> list[float]:
        # avoid singularity in upright position
        if delta < 1e-6:
            return [1,0,0, 0,1,0, 0,0,1]
        R11 = 1 + delta_x ** 2 / delta ** 2 * (1 - np.cos(delta))
        R12 = delta_x * delta_y / delta ** 2 * (1 - np.cos(delta))
        R13 = - delta_x / delta * np.sin(delta)
        R21 = R12
        R22 = 1 + delta_y ** 2 / delta ** 2 * (1 - np.cos(delta))
        R23 = - delta_y / delta * np.sin(delta)
        R31 = - R13
        R32 = - R23
        R33 = 1 + (1 - np.cos(delta))
        return [R11, R12, R13,
                R21, R22, R23,
                R31, R32, R33]

    def get_transformation_vector(self, delta_x: float, delta_y: float, delta: float) -> Vector3:
        # avoid singularity in upright position
        if delta < 1e-6:
            return Vector3(x=0.0, y=0.0, z=self.segment_length)
        # simply calculate each segment
        factor = self.segment_length / delta ** 2
        # transformation vector from some scientific paper
        x_totip = factor * (1 - np.cos(delta)) * delta_x
        y_totip = factor * (1 - np.cos(delta)) * delta_y
        z_totip = factor * np.sin(delta) * delta
        return Vector3(x=x_totip, y=y_totip, z=z_totip)

def main():
    rclpy.init()
    mynode = G2X_mag_tip()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()