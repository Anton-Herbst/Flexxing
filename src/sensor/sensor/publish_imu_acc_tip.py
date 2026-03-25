#!/usr/bin/env python3

"""
* This script simply grabs raw sensor data (imu linear acceleration) and transforms it to the local coordinate system.
* Before this is run calibrations should have been completed.
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # specifics of transmission protocol for ros topics
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors 
from geometry_msgs.msg import Accel                                 # datatype used by imu raw (accel) topic
import os                                                           # operatin system, needed to find path to local yaml
import yaml                                                         # handling yaml file

class Publisher_imu_acc_tip(Node):

    # * function called on creation
    def __init__(self):
        
        # "it insists upon itself"
        super().__init__('publish_imu_acc_tip')
        # kept constant, identifier for the sensor at the tip of the robot
        self.tip = 1 

        # * ROS related
        # micro ros cant guarantee loss free data transmission so this is to match the expected QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # create subscriber to get new data
        self.subscription_acc       = self.create_subscription(Accel, f'/teensy_hub/imu_data_{self.tip}', self.callback_imu_acc, qos)
        #create publisher to release transformed Vector
        self.publisher              = self.create_publisher(Vector3, '/pc/transformed_imu_acc_tip', 10)

        # * Parameter from calibration (local YAML)
        # read in the config file with the calibrated values
        config_file_path_imu = os.path.expanduser('~/.ros/calibration/imu_cal_tip.yaml')
        with open(file = config_file_path_imu, mode = 'r') as file_handle:
            extracted_data_imu = yaml.safe_load(stream = file_handle)
        # dict of transformation matrixes, use numpy for easier matrix multiplication
        self.matrix_rot = np.array(extracted_data_imu['imu_acc'])
        
    # * callback function of sensors
    # whenever new data arrives do some math and publish the results straightaway
    def callback_imu_acc(self, msg: Accel) -> None: self.transform(msg.linear)

    # * function tasked with transformation 
    # going from the sensors defined system to the local robots system
    def transform(self, raw: Vector3) -> None:
        # convert raw measured Vector3 type into a numpy array
        measured = np.array([raw.x, raw.y, raw.z])
        # use numpy for matrix multiplication (coordinate transformation)
        transformed = self.matrix_rot @ measured
        # now normalize it
        normalized = transformed / np.linalg.norm(transformed)
        # convert to publishable format
        msg_pub = Vector3(x = normalized[0].item(), y = normalized[1].item(), z = normalized[2].item())
        # fire it away
        self.publisher.publish(msg_pub)


def main():
    rclpy.init()
    mynode = Publisher_imu_acc_tip()
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