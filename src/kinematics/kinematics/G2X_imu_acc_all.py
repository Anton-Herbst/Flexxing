#!/usr/bin/env python3

"""
! only needed for visualization, not for actual control procedure
* This script is partial forward kinematics of the PCC robot.
* Its purpose is to convert incoming generalized coordinates to the endeffector position.
* configuration G -> endeffector X
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray

class Forward_PCC_G2X(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('forward_PCC_G2X_imu_all')

        # * ROS related
        self.subscription_bot = self.create_subscription(
            msg_type    = Float64MultiArray,
            topic       = '/pc/gen_coords_imu_acc_bot',
            callback    = lambda msg: self.callback_gen_coords(msg, 'bot'),
            qos_profile = 10)
        self.subscription_top = self.create_subscription(
            msg_type    = Float64MultiArray,
            topic       = '/pc/gen_coords_imu_acc_top',
            callback    = lambda msg: self.callback_gen_coords(msg, 'top'),
            qos_profile = 10)
        self.publisher_endeffector = self.create_publisher(Vector3, '/pos/endeffector', 10)

        # * Parameter
        self.segment_length = self.declare_parameter('L_segment', 0.12).value

        # * Variables
        self.matrix_rotate_bot = np.eye(3)
        self.transform = {'bot': Vector3(), 'top': Vector3()}

    # * callback on receiving new generalized coordinates
    def callback_gen_coords(self, msg: Float64MultiArray, segment: str) -> None:
        # read incoming generalized coordinates
        delta_x, delta_y, delta = msg.data
        # compute and save transformation vector for this segment
        self.transform[segment] = self.get_transformation_vector(delta_x, delta_y, delta)
        # compute and save rotation matrix for bot segment
        if segment == 'bot':
            self.matrix_rotate_bot = self.get_rotation_matrix(delta_x, delta_y, delta)
        # publish updated endeffector position
        self.publisher_endeffector.publish(self.get_endeffector_position())

    def get_rotation_matrix(self, delta_x: float, delta_y: float, delta: float) -> np.ndarray:
        # avoid singularity in upright position
        if delta < 1e-6:
            return np.eye(3)

        c = np.cos(delta)
        s = np.sin(delta)
        d2 = delta ** 2

        R11 = 1 + (delta_x ** 2 / d2) * (c - 1)
        R12 = (delta_x * delta_y / d2) * (c - 1)
        R13 = (delta_x / delta) * s

        R21 = R12
        R22 = 1 + (delta_y ** 2 / d2) * (c - 1)
        R23 = (delta_y / delta) * s

        R31 = -R13
        R32 = -R23
        R33 = c

        return np.array([
            [R11, R12, R13],
            [R21, R22, R23],
            [R31, R32, R33]
        ])

    def get_transformation_vector(self, delta_x: float, delta_y: float, delta: float) -> Vector3:
        # avoid singularity in upright position
        if delta < 1e-6:
            return Vector3(x=0.0, y=0.0, z=self.segment_length)
        factor = self.segment_length / delta ** 2
        x = factor * (1 - np.cos(delta)) * delta_x
        y = factor * (1 - np.cos(delta)) * delta_y
        z = factor * np.sin(delta) * delta
        return Vector3(x=x, y=y, z=z)

    def get_endeffector_position(self) -> Vector3:
        t_bot = np.array([self.transform['bot'].x, self.transform['bot'].y, self.transform['bot'].z])
        t_top = np.array([self.transform['top'].x, self.transform['top'].y, self.transform['top'].z])
        position = t_bot + self.matrix_rotate_bot @ t_top
        return Vector3(x=position[0].item(), y=position[1].item(), z=position[2].item())

def main():
    rclpy.init()
    mynode = Forward_PCC_G2X()
    try:
        rclpy.spin(mynode)
    except KeyboardInterrupt:
        pass
    finally:
        mynode.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()