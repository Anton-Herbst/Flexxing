#!/usr/bin/env python3

"""
* This file looks for the absolute distance to the targeted
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
from std_msgs.msg import Float64                                    # datatype used for singular float

class X_error(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('X_factor')

        # * Variables for this Node
        self.last_pos = Vector3()
        self.last_traj= Vector3()
        hz = 60

        # * ROS related
        # subscribe to the cartesian coordinates of endeffector and target trajectory
        self.subscription_endeffector = self.create_subscription(Vector3, '/pos/endeffector', self.callback_endeffector, 10)
        self.subscription_trajectory = self.create_subscription(Vector3, '/pos/trajectory', self.callback_trajectory, 10)
        self.publish_error = self.create_publisher(Float64, '/pos/cartesian_error', 10)
        self.publish_timer = self.create_timer(1/hz, self.callback_timer)

    # * callback functions
    def callback_endeffector(self, msg: Vector3) -> None: self.last_pos = msg
    def callback_trajectory(self, msg: Vector3) -> None: self.last_traj = msg
    def callback_timer(self) -> None: self.calculate_and_publish()

    # * help function
    def calculate_and_publish(self) -> None:
        delta_x = self.last_pos.x - self.last_traj.x
        delta_y = self.last_pos.y - self.last_traj.y
        delta_z = self.last_pos.z - self.last_traj.z
        absolute_error = np.linalg.norm([delta_x, delta_y, delta_z])
        msg = Float64()
        msg.data = absolute_error
        self.publish_error.publish(msg)

def main():
    rclpy.init()
    mynode = X_error()
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