#!/usr/bin/env python3

"""
* This script simply grabs raw sensor data (imu linear acceleration) from all sensors and transforms it
* to the local coordinate system.
* Before this is run all calibrations should have been completed.
"""

import numpy as np                                                  # for math stufft
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # specifics of transmission protocol for ros topics
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors
import os                                                           # operatin system, needed to find path to local yaml
import yaml                                                         # handling yaml file

class Publisher_mag_all(Node):

    # * function called on creation
    def __init__(self):
        # "it insists upon itself"
        super().__init__('publish_mag_all')

        # * ROS related
        # micro ros cant guarantee loss free data transmission so this is to match the expected QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # create subscribers to get the vector from the magnetic sensors
        self.subscription_1 = self.create_subscription(Vector3, '/teensy_hub/mag_data_1', lambda msg: self.callback_mag(msg, 1), qos)
        self.subscription_2 = self.create_subscription(Vector3, '/teensy_hub/mag_data_2', lambda msg: self.callback_mag(msg, 2), qos)
        self.subscription_3 = self.create_subscription(Vector3, '/teensy_hub/mag_data_3', lambda msg: self.callback_mag(msg, 3), qos)
        self.subscription_4 = self.create_subscription(Vector3, '/teensy_hub/mag_data_4', lambda msg: self.callback_mag(msg, 4), qos)
        # create publishers to publish the transformed vectors
        self.publisher_select = {
            1: self.create_publisher(Vector3, '/pc/transformed_mag_1', 10),
            2: self.create_publisher(Vector3, '/pc/transformed_mag_2', 10),
            3: self.create_publisher(Vector3, '/pc/transformed_mag_3', 10),
            4: self.create_publisher(Vector3, '/pc/transformed_mag_4', 10),}

        # * Parameter from calibration (local YAML)
        # read in the config file with the calibrated values
        config_file_path = os.path.expanduser('~/.ros/calibration/mag_cal_all.yaml')
        with open(file = config_file_path, mode = 'r') as file_handle:
            extracted_data = yaml.safe_load(stream = file_handle)
        # dict of transformation matrices, use numpy for easier matrix multiplication
        self.matrix_rot = {
            1: np.array(extracted_data['magnetometer_calibration_all'][1]),
            2: np.array(extracted_data['magnetometer_calibration_all'][2]),
            3: np.array(extracted_data['magnetometer_calibration_all'][3]),
            4: np.array(extracted_data['magnetometer_calibration_all'][4]),}

    # * callback function of sensors
    # whenever new data arrives do some math and publish the results straightaway
    def callback_mag(self, msg: Vector3, key: int) -> None:
        self.transform(key, msg)

    # * function tasked with transformation
    # going from the sensors defined system to the local robots system
    def transform(self, key: int, raw: Vector3) -> None:
        # convert raw measured Vector3 type into a numpy array
        measured = np.array([raw.x, raw.y, raw.z])
        # use numpy for matrix multiplication (coordinate transformation)
        transformed = self.matrix_rot[key] @ measured
        # now normalize it
        normalized = transformed / np.linalg.norm(transformed)
        # convert to publishable format
        msg_pub = Vector3(x=normalized[0].item(), y=normalized[1].item(), z=normalized[2].item())
        # publish to the according topic
        self.publisher_select[key].publish(msg_pub)

def main():
    rclpy.init()
    mynode = Publisher_mag_all()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()