#! /usr/bin/env python3

"""
* This script is partial inverse kinematics of the PCC robot.
* It is important to note that this only works when the bottom segment is straigt !
* Its purpose is to convert incoming generalized coordinates from the tip sensor to tendon lengths (only top segment).
* configuration G -> tendon length L
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from std_msgs.msg import Float64MultiArray                          # datatype used for multiple floats

class Inverse_PCC_G2L_tip(Node):
    
    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('inverse_PCC_G2L_tip')

        # * ROS related
        # subscribe to incoming generalized coordinates for the tip segment from sensor/gen_.._imu_acc_tip
        self.subscription = self.create_subscription( 
            msg_type    = Float64MultiArray, 
            topic       = '/pc/gen_coords_imu_acc_tip', 
            callback    = self.callback_gen_coords_tip, 
            qos_profile = 10 )
        # publisher giving out current real tendon lengths
        self.publisher = self.create_publisher( Float64MultiArray, '/pc/tendon_length_tip', 10 )
        
        # * Geometric Parameters
        # the length of the segment (since we only look at the upper point)
        self.segment_length = self.declare_parameter('L_segment', 0.12).value
        # distance to the middle arc
        self.d = self.declare_parameter('d', 0.01).value
        # topper rotation
        self.yaw_offset = np.deg2rad(-60)

    # * callback on receiving new info
    def callback_gen_coords_tip(self, msg: Float64MultiArray) -> None: 
        # extract incoming data
        delta_x, delta_y, _ = msg.data
        # calculate the tendon lengths
        l4, l5, l6 = self.inverse_kinematics_G2L_top(delta_x, delta_y)
        # publish the tendon lengths
        self.publish_tendon_lengths(l4, l5, l6)

    # * function to calculate tendon lengths from generalized coordinates
    def inverse_kinematics_G2L_top(self, delta_x: float, delta_y: float) -> tuple[float, float, float]:
        # the tendon lengths are calculated from the generalized coordinates and the segment length
        l4 = self.segment_length + self.d * (delta_x * np.cos(self.yaw_offset)           + delta_y * np.sin(self.yaw_offset))
        l5 = self.segment_length + self.d * (delta_x * np.cos(self.yaw_offset+2/3*np.pi) + delta_y * np.sin(self.yaw_offset+2/3*np.pi))
        l6 = self.segment_length + self.d * (delta_x * np.cos(self.yaw_offset+4/3*np.pi) + delta_y * np.sin(self.yaw_offset+4/3*np.pi))
        return l4, l5, l6 

    # * function pushing data to publisher
    def publish_tendon_lengths(self, l4: float, l5: float, l6: float) -> None:
        # prepare a message
        msg = Float64MultiArray()
        # load the tendon lengths into the message
        msg.data = [l4, l5, l6]
        # publish the message
        self.publisher.publish(msg)
        #also print it so i can see it in the terminal
        self.get_logger().info(f'Tendon lengths: l4={l4:.4f}, l5={l5:.4f}, l6={l6:.4f}')

def main():
    rclpy.init()
    mynode = Inverse_PCC_G2L_tip()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()