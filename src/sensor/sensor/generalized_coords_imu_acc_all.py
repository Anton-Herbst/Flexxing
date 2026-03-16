#!/usr/bin/env python3

"""
* This file grabs the sensor's transformed vector. From it the generalized coordinates are published,
* which can then be used to get each segments rotational matrix and transformation hence the robots full posture 
* (forward kinematics will handle that)

TODO:
- make the 2nd and 4th sensor not publish, they are only needed to middle the resulting angles
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
from std_msgs.msg import Float64MultiArray                          # datatype used for publishing PCC values

class Gen_coords_imu_acc(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('gen_coords_imu_acc')

        # * Parameter for this node
        # keep a dict of the last incoming (global) angles from the sensors so their difference (local angles) can later be calculated
        self.phi_global = { i: 0.0 for i in range(1,5) }
        self.theta_global = { i: 0.0 for i in range(1,5) }
        # middled results
        self.phi_global_middled = { 'bot': 0.0, 'top': 0.0 }

        # * ROS related
        # subscribe to the transformed vector topic for each sensor
        self.subscription_1 = self.create_subscription(Vector3, '/pc/transformed_imu_acc_1', lambda msg: self.callback_acc(msg, 1), 10)
        self.subscription_2 = self.create_subscription(Vector3, '/pc/transformed_imu_acc_2', lambda msg: self.callback_acc(msg, 2), 10)
        self.subscription_3 = self.create_subscription(Vector3, '/pc/transformed_imu_acc_3', lambda msg: self.callback_acc(msg, 3), 10)
        self.subscription_4 = self.create_subscription(Vector3, '/pc/transformed_imu_acc_4', lambda msg: self.callback_acc(msg, 4), 10)
        # create a publisher dict to select where to publish the results
        self.publisher_select = {
            # they will carry an array which consist of the [delta_x, delta_y and their norm delta] for each sensor
            'bot': self.create_publisher( Float64MultiArray,'/pc/gen_coords_imu_acc_bot', 10),
            'top': self.create_publisher( Float64MultiArray,'/pc/gen_coords_imu_acc_top', 10),}
        # create a timer to publish the local angles at a fixed rate 
        hz = 20
        self.publishing_timer = self.create_timer(1/hz, self.publish_local_angles) 

    # * callback on receiving new info
    def callback_acc(self, msg: Vector3, sensor: int) -> None:
        # save the last incoming angles from the sensors vector to the global coordinate system
        self.phi_global[sensor] = self.get_global_bendingplane_angle(msg)
        self.theta_global[sensor] = self.get_global_vertical_angle(msg)
        # middle the global angles of the 1st and 2nd as well as the 3rd and 4th sensor to get more stable results for the local angles
        self.phi_global_middled['top'] = (self.phi_global[1] + self.phi_global[2]) / 2
        self.phi_global_middled['bot'] = (self.phi_global[3] + self.phi_global[4]) / 2

    # * function determing plane in which the robot is bent
    def get_global_bendingplane_angle(self, vec: Vector3) -> float:
        # plane is described by rotation around z
        phi = np.arctan2(vec.y, vec.x) % (2 * np.pi)
        # transform back to python float and return it
        return phi.item()
    
    # * function calculating angular offset to the rooms z-axis
    def get_global_vertical_angle(self, vec: Vector3) -> float:
        # the curvate is defined as k= 1/r = °/l and since we are interested in delta_x and delta_y l will be crossed out
        planar_axis = np.sqrt(vec.x ** 2 + vec.y ** 2)
        theta = np.arctan2(planar_axis, vec.z)
        # return a python float
        return theta.item()
    
    # * callback of the timer 
    def publish_local_angles(self):
        # calculate the local angles from the global angles on callback
        phi_local, theta_local = self.get_local_angles()     
        # publish the local angles for each sensor
        for segment in 'bot', 'top':
            # prepare a message for each sensor
            msg = Float64MultiArray()
            # fill its data field with the according generalized coordinates derived from local angles  
            msg.data = list(self.get_generalized_coordinates(theta_local[segment], phi_local[segment]))
            # now publish the message containing [delta_x, delta_y and delta] for the segment to the right topic
            self.publisher_select[segment].publish(msg)
            
    
    # * function calculating local angles from the global angles 
    def get_local_angles(self) -> tuple[dict[str, float], dict[str, float]]:
        # create dicts for local angles
        phi_local = { 'bot': 0.0, 'top': 0.0 }
        theta_local = { 'bot': 0.0, 'top': 0.0 }
        # the first sensor is the reference so its local angle is the same as its global angle
        phi_local['bot'] = self.phi_global_middled['bot']
        theta_local['bot'] = self.theta_global[3]
        # local angles are calculated by substracting the previous sensors global angle from the current sensors global angle
        phi_local['top'] = (self.phi_global_middled['top'] - self.phi_global_middled['bot']) % (2 * np.pi)
        theta_local['top'] = self.theta_global[4]
        # return the local angles as dicts
        return phi_local, theta_local
    
    # * function calculating the generalized coordinates from the local angles
    def get_generalized_coordinates(self, theta:float, phi: float) -> tuple[float, float, float]:
        # this only works with local angles, which we calculate beforehand
        delta_x = theta * np.cos(phi)
        delta_y = theta * np.sin(phi)
        return delta_x, delta_y, theta
    
def main():
    rclpy.init()
    mynode = Gen_coords_imu_acc()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()