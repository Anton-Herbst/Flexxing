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
from geometry_msgs.msg import Accel                                 # datatype used by imu raw (accel) topic
import os                                                           # operatin system, needed to find path to local yaml
import yaml                                                         # handling yaml file

class Publisher_imu_acc_all(Node):

    # * function called on creation
    def __init__(self):
        # "it insists upon itself"
        super().__init__('publish_imu_acc_all')

        # * ROS related
        # micro ros cant guarantee loss free data transmission so this is to match the expected QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # create subscribers to get the vector from the imu sensor
        self.subscription_1  = self.create_subscription(Accel, '/teensy_hub/imu_data_1', self.callback_imu_acc_1, qos)
        self.subscription_2  = self.create_subscription(Accel, '/teensy_hub/imu_data_2', self.callback_imu_acc_2, qos)
        self.subscription_3  = self.create_subscription(Accel, '/teensy_hub/imu_data_3', self.callback_imu_acc_3, qos)
        self.subscription_4  = self.create_subscription(Accel, '/teensy_hub/imu_data_4', self.callback_imu_acc_4, qos)
        # create publisher dict to select the sensor which is being transformed and published
        self.publisher_select = {
            1: self.create_publisher(Vector3, '/pc/transformed_imu_acc_1', 10 ),
            2: self.create_publisher(Vector3, '/pc/transformed_imu_acc_2', 10 ),
            3: self.create_publisher(Vector3, '/pc/transformed_imu_acc_3', 10 ),
            4: self.create_publisher(Vector3, '/pc/transformed_imu_acc_4', 10 ),}
        
        # * Parameter from calibration (local YAML)
        # read in the config file with the calibrated values
        config_file_path_imu = os.path.expanduser('~/.ros/calibration/imu_cal_all.yaml')
        with open(file = config_file_path_imu, mode = 'r') as file_handle:
            extracted_data = yaml.safe_load(stream = file_handle)
        # dict of transformation matrixes, use numpy for easy matrix multiplication
        self.matrix_rot = {
            1: np.array(extracted_data['imu_acc'][1]),
            2: np.array(extracted_data['imu_acc'][2]),
            3: np.array(extracted_data['imu_acc'][3]),
            4: np.array(extracted_data['imu_acc'][4]),}

    # * callback function of sensor
    # immediatly transform raw data and publish resulting vector for all to see
    def callback_imu_acc_1(self, msg: Accel) -> None:  self.transform(1, msg.linear)
    def callback_imu_acc_2(self, msg: Accel) -> None:  self.transform(2, msg.linear)
    def callback_imu_acc_3(self, msg: Accel) -> None:  self.transform(3, msg.linear)
    def callback_imu_acc_4(self, msg: Accel) -> None:  self.transform(4, msg.linear)
    
    # * function tasked with transformation and publishing
    def transform(self, key:int, raw: Vector3) -> None:
        # convert raw measured Vector3 type into a numpy array
        measured = np.array([raw.x, raw.y, raw.z])
        # use numpy for matrix multiplication (coordinate transformation)
        out = self.matrix_rot[key] @ measured
        # now normalize it
        normalized = out / np.linalg.norm(out)
        # convert back to python floats with a vector struct
        msg_calib = Vector3(x = normalized[0].item(), y = normalized[1].item(), z = normalized[2].item())
        # publish this vector in the according topic
        self.publisher_select[key].publish(msg_calib)

def main():
    rclpy.init()
    mynode = Publisher_imu_acc_all()
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