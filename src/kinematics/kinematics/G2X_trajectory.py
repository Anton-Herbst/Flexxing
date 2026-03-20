#!/usr/bin/env python3

"""
* The purpose of this script is to publish the cartesian coordinates of the trajectory for later plotting.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray

class G2X_imu_acc_tip(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('forward_PCC_G2X_trajectory')

        # * ROS related
        self.subscription = self.create_subscription(Float64MultiArray, '/pc/controller/trajectory', self.callback_gen_coords, 10)
        self.publisher_endeffector = self.create_publisher(Vector3, '/pos/trajectory', 10)

        # * Parameter
        self.segment_length = self.declare_parameter('L_segment', 0.12).value

    # * callback on receiving new generalized coordinates
    def callback_gen_coords(self, msg: Float64MultiArray) -> None:
        # read incoming configuration
        delta_x_bot, delta_y_bot, delta_x_top, delta_y_top = msg.data
        # calculate transformation vectors for each segment
        t_bot = self.get_transformation_vector(delta_x_bot, delta_y_bot)
        t_top = self.get_transformation_vector(delta_x_top, delta_y_top)
        # get rotation matrix of bottom segment
        R_bot = self.get_rotation_matrix(delta_x_bot, delta_y_bot)
        # chain: endeffector = t_bot + R_bot @ t_top
        t_bot_np = np.array([t_bot.x, t_bot.y, t_bot.z])
        t_top_np = np.array([t_top.x, t_top.y, t_top.z])
        endeffector = t_bot_np + R_bot @ t_top_np
        # prepare and publish message
        msg_pub = Vector3(x=endeffector[0].item(), y=endeffector[1].item(), z=endeffector[2].item())
        self.publisher_endeffector.publish(msg_pub)

    def get_rotation_matrix(self, delta_x: float, delta_y: float) -> np.ndarray:
        delta = np.sqrt(delta_x ** 2 + delta_y ** 2)
        # avoid singularity in upright position
        if delta < 1e-6:
            return np.eye(3)
        # calculate each entry
        R11 = 1 + delta_x ** 2 / delta ** 2 * (1 - np.cos(delta))
        R12 = delta_x * delta_y / delta ** 2 * (1 - np.cos(delta))
        R13 = - delta_x / delta * np.sin(delta)
        R21 = R12
        R22 = 1 + delta_y ** 2 / delta ** 2 * (1 - np.cos(delta))
        R23 = - delta_y / delta * np.sin(delta)
        R31 = - R13
        R32 = - R23
        R33 = 1 + (1 - np.cos(delta))
        return np.array([[R11, R12, R13],
                         [R21, R22, R23],
                         [R31, R32, R33]])

    def get_transformation_vector(self, delta_x: float, delta_y: float) -> Vector3:
        delta = np.sqrt(delta_x ** 2 + delta_y ** 2)
        # avoid singularity in upright position
        if delta < 1e-6:
            return Vector3(x=0.0, y=0.0, z=self.segment_length)
        # calculate transformation vector
        factor = self.segment_length / delta ** 2
        x_totip = factor * (1 - np.cos(delta)) * delta_x
        y_totip = factor * (1 - np.cos(delta)) * delta_y
        z_totip = factor * np.sin(delta) * delta
        return Vector3(x=x_totip, y=y_totip, z=z_totip)

def main():
    rclpy.init()
    mynode = G2X_imu_acc_tip()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()