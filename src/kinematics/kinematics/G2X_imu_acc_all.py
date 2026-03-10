#! /usr/bin/env python3

"""
! only needed for visualization, not for actual control procedure
* This script is partial forward kinematics of the PCC robot.
* Its purpose is to convert incoming generalized coordinates to the endeffector position.
* configuration G -> endeffector X
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
from std_msgs.msg import Float64MultiArray                          # datatype used for multiple floats

class Forward_PCC_G2X(Node):
    
    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('forward_PCC_G2X')

        # * ROS related
        # subscribe to incoming generalized coordinates for each segment
        self.subscription_bot = self.create_subscription(
            msg_type    =  Float64MultiArray,
            topic       = '/pc/gen_coords_imu_acc_bot',
            callback    = lambda msg: self.gen_coords_imu_acc(msg, 'bot'),
            qos_profile = 10 )
        self.subscription_top = self.create_subscription(
            msg_type    =  Float64MultiArray,
            topic       = '/pc/gen_coords_imu_acc_top',
            callback    = lambda msg: self.gen_coords_imu_acc(msg, 'top'),
            qos_profile = 10 )
        # publisher giving out current real position
        self.publisher_select = {
            'transform': {
                'bot': self.create_publisher( Vector3, '/pos/transform_bot', 10 ),
                'top': self.create_publisher( Vector3, '/pos/transform_top', 10 ),},
            'rotate': {
                'bot': self.create_publisher( Float64MultiArray, '/pos/rotate_bot', 10 ),
                'top': self.create_publisher( Float64MultiArray, '/pos/rotate_top', 10 ),}}
        self.publisher_endeffector = self.create_publisher( Vector3, '/pos/endeffector', 10 )

        # * Parameter
        self.segment_length = self.declare_parameter('L_segment', 0.12).value
        self.matrix_rotate_bot = np.eye(3)
        self.transform = {'bot': Vector3(), 'top': Vector3()}

    # * callback on receiving new generalized coordinates
    def gen_coords_imu_acc(self, msg: Float64MultiArray, segment: str) -> None:
        # read incoming target positions
        delta_x, delta_y, delta = msg.data
        # prepare a message
        msg_rotate = Float64MultiArray()
        # get the transformation vector for the according segment
        msg_rotate.data = self.get_rotation_matrix(delta_x, delta_y, delta)
        # publish it
        self.publisher_select['rotate'][segment].publish(msg_rotate)
        # also save it 
        if segment == 'bot':
            self.matrix_rotate_bot = np.array(msg_rotate.data).reshape(3, 3)
        # prepare another message
        msg_transform = Vector3()
        # get the transformation vector for the according segment
        msg_transform = self.get_transformation_vector(delta_x, delta_y, delta)
        # publish it
        self.publisher_select['transform'][segment].publish(msg_transform)
        # also save it
        self.transform[segment] = msg_transform
        # update the endeffector position
        self.publisher_endeffector.publish(self.get_endeffector_position())

    def get_rotation_matrix(self, delta_x: float, delta_y: float, delta: float) -> list[float]:
        # avoid singularity in upright position
        if delta < 1e-6:
            return [1,0,0, 0,1,0, 0,0,1]  # just return identity matrix
        # simply calculate each segment
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
            return Vector3(x=0, y=0, z=self.segment_length)  # just return straight position
        # simply calculate each segment
        factor = self.segment_length / delta ** 2
        x = factor * (1-np.cos(delta)) * delta_x
        y = factor * (1-np.cos(delta)) * delta_y
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
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
