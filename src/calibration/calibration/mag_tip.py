#!/usr/bin/env python3

"""
* This script calibrates the magnetic sensor at the tip relative to the IMU on the same PCB.
* Both sensors share the same rigid board but their chip axes point in different directions.
* This script finds the rotation matrix to go from mag frame to IMU frame.
* Having created that matrix it can be used with th IMUs calibration to 
*
* Once known, the full chain to robot frame is:
*   R_mag_to_robot = R_imu_to_robot (known, from IMU cal) @ R_mag_to_imu
*
* -------------------------------------------------------------------------------------------------
* A rotation matrix has 9 parameters. With 3 measurements of both sensors vectors
* at 3 different poses, the system is exactly determined:
*
*   At each pose i:   g_i = R_mag_to_imu @ B_i
*
*   Stacked:   G = R_mag_to_imu @ B
*   Where:     G = [g1 | g2 | g3]   (3x3, IMU gravity vectors as columns)
*              B = [B1 | B2 | B3]   (3x3, mag field vectors as columns)
*
*   Solution:  R_mag_to_imu = G @ inv(B)

TODO list
!- confirm it works with visuals
"""

import numpy as np                                                  # useful for mathematics
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from rclpy.parameter import Parameter                               # be able to use params
from rcl_interfaces.msg import ParameterDescriptor                  # describe them if i forget or theyre named too short
from soro_msgs.msg import ServoCommands                             # datatype for publishing servos

from geometry_msgs.msg import Accel                               # datatype acceleration (divided in vecotrs for linear and rotational)
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors
import yaml                                                         # access to yaml file
import os, shutil                                                   # operating system, needed to write into another file
from ament_index_python.packages import get_package_share_directory # gets path to this package independent from each pc
import pprint                                                       # pretty printing, better visuals for list
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # specifics of transmission protocol for ros topics


class CalibrateMagneticSensor(Node):

    # * function called on creation
    def __init__(self):
        # "it insists upon itself"
        super().__init__('mag_sensor_calibration_tip')
        # kept constant, identifier for the sensor at the tip of the robot
        self.tip = 1

        # * ROS related init
        # micro ros cant guarantee loss free data transmission so this is to match the expected QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # create a subscriber to get the vector from the magnetic sensor at the tip
        self.subscription = self.create_subscription( Vector3, f'/teensy_hub/mag_data_{self.tip}', self.mag_callback, qos)
        # subscribe to the IMU linear acceleration
        self.imu_sub = self.create_subscription(Accel, f'/teensy_hub/imu_data_{self.tip}', self.imu_callback, qos)
        # link into the servo topic to command the robots actuators
        self.servo_pub = self.create_publisher(ServoCommands, '/teensy_hub/servo_pos', 10)
        # create a timer to oversee progress
        self.planner = self.create_timer(
            timer_period_sec    = 0.1,
            callback            = self.planner_callback,)
        # mark down the starting point of the timer
        self.start_time = self.planner.clock.now().nanoseconds

        # * Parameters from pose_cal.yaml
        self.declare_parameters(
            namespace='',
            parameters=[
                ('upright',    Parameter.Type.INTEGER_ARRAY, ParameterDescriptor(description='Servo values for the upright pose.')),
                ('rotated_1',  Parameter.Type.INTEGER_ARRAY, ParameterDescriptor(description='Servo values for the first rotated pose.')),
                ('rotated_2',  Parameter.Type.INTEGER_ARRAY, ParameterDescriptor(description='Servo values for the second rotated pose.')),
            ])

        # * Parameters for this node
        # how long to wait until the robot is stationary (no swinging) in seconds
        self.wait          = 3
        # how long it takes to reach the new position
        self.drive         = 1
        # how long to sample at each pose
        self.sampling_time = 2
        # boolean to decide whether to catch sensor data
        self.sampling      = False
        # lists to store samples during the active window
        self.mag_samples   = []
        self.imu_samples   = []
        # boolean to exit cleanly
        self.done          = False
        # columns of G and B built up pose by pose
        # each pose appends one column: g_i to G_cols, B_i to B_cols
        self.G_cols        = []   # gravity vectors in IMU frame
        self.B_cols        = []   # field vectors in mag frame

    # * callback function of the planning timer
    def planner_callback(self) -> None:
        # simply skip if the program is done and the node is somehow still spinning
        if self.done is True:   return
        # calculate elapsed time in seconds
        delta_t = (self.planner.clock.now().nanoseconds - self.start_time) / 1e9
        # one period = drive + wait + sample for one pose
        period = self.drive + self.wait + self.sampling_time

        if delta_t < self.drive + self.wait:
            # drive to upright and hold until stationary
            self.command_servos('upright')
            self.get_logger().info('Driving to upright position.', once=True)

        elif delta_t < period:
            # sample pose 1
            self.sampling = True
            self.get_logger().info('Collecting data at pose 1 (upright).', once=True)

        elif delta_t < period + self.drive + self.wait:
            if self.sampling is True:
                self.sampling = False
                self.get_logger().info('Stopped collecting (pose 1).', once=True)
                self.save_pose()
            # drive to first rotated position
            self.command_servos('rotated_1')
            self.get_logger().info('Driving to rotated position 1.', once=True)

        elif delta_t < 2 * period:
            # sample pose 2
            self.sampling = True
            self.get_logger().info('Collecting data at pose 2 (rotated_1).', once=True)

        elif delta_t < 2 * period + self.drive + self.wait:
            if self.sampling is True:
                self.sampling = False
                self.get_logger().info('Stopped collecting (pose 2).', once=True)
                self.save_pose()
            # drive to second rotated position
            self.command_servos('rotated_2')
            self.get_logger().info('Driving to rotated position 2.', once=True)

        elif delta_t < 3 * period:
            # sample pose 3
            self.sampling = True
            self.get_logger().info('Collecting data at pose 3 (rotated_2).', once=True)

        elif delta_t > 3 * period:
            if self.sampling is True:
                self.sampling = False
                self.get_logger().info('Stopped collecting (pose 3).', once=True)
                self.save_pose()
            # all three poses collected — solve and write
            self.solve_and_write()
            self.done = True

    # * evaluate current samples and store one column into G and B
    def save_pose(self) -> None:
        g = self.evaluate_imu_samples()
        B = self.evaluate_mag_samples()
        self.G_cols.append(g)
        self.B_cols.append(B)
        self.get_logger().info(
            f'Pose {len(self.G_cols)} saved. '
            f'g={np.round(g, 3).tolist()}  B={np.round(B, 3).tolist()}')

    # * solve R_mag_to_imu = G @ inv(B) and write the final matrix
    def solve_and_write(self) -> None:
        # assemble the 3x3 matrices column by column
        G = np.column_stack(self.G_cols)   # [g1 | g2 | g3]
        B = np.column_stack(self.B_cols)   # [B1 | B2 | B3]

        # sanity check: B must be invertible — columns must be linearly independent
        # a small determinant means the three poses were nearly coplanar
        det_B = np.linalg.det(B)
        if abs(det_B) < 0.1:
            self.get_logger().error(
                f'Mag measurement matrix is near-singular (det={det_B:.4f}). '
                'The three poses are too similar — choose more distinct orientations.')
            return

        # direct solution: R = G @ inv(B)
        R_raw = G @ np.linalg.inv(B)

        # R_raw may deviate slightly from a perfect rotation matrix due to sensor noise
        # project onto SO(3) via SVD to get the nearest valid rotation matrix
        U, _, Vt = np.linalg.svd(R_raw)
        R_mag_to_imu = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
        self.get_logger().info('R_mag_to_imu:\n' + pprint.pformat(R_mag_to_imu.tolist()))

        # chain with the known R_imu_to_robot
        R_imu_to_robot = self.load_imu_calibration()
        R_mag_to_robot = R_imu_to_robot @ R_mag_to_imu
        self.get_logger().info('R_mag_to_robot:\n' + pprint.pformat(R_mag_to_robot.tolist()))

        # write to yaml
        self.write_down_matrix(R_mag_to_robot)

    # * average IMU gravity samples → unit numpy vector
    def evaluate_imu_samples(self) -> np.ndarray:
        if not self.imu_samples:
            self.get_logger().warn('No IMU samples — returning default.')
            return np.array([0.0, 0.0, 1.0])
        vectors = np.array(self.imu_samples)
        mean    = np.mean(vectors, axis=0)
        self.imu_samples.clear()
        return mean / np.linalg.norm(mean)

    # * average mag samples → unit numpy vector
    def evaluate_mag_samples(self) -> np.ndarray:
        if not self.mag_samples:
            self.get_logger().warn('No mag samples — returning default.')
            return np.array([1.0, 0.0, 0.0])
        vectors = np.array(self.mag_samples)
        mean    = np.mean(vectors, axis=0)
        self.mag_samples.clear()
        return mean / np.linalg.norm(mean)

    # * callback function of the tip magnetic sensor
    def mag_callback(self, msg: Vector3) -> None:
        if self.sampling:
            self.mag_samples.append([msg.x, msg.y, msg.z])

    # * callback function of the IMU gravity vector
    def imu_callback(self, msg: Vector3) -> None:
        if self.sampling:
            self.imu_samples.append([msg.x, msg.y, msg.z])

    # * function to move the robot
    def command_servos(self, pose_name: str) -> None:
        servo_msg = ServoCommands()
        servo_msg.servo_micros = self.get_parameter(pose_name).get_parameter_value().integer_array_value
        self.servo_pub.publish(servo_msg)

    # * load the known R_imu_to_robot calibration matrix from yaml
    def load_imu_calibration(self) -> np.ndarray:
        path = os.path.expanduser('~/.ros/calibration/imu_cal_tip.yaml')
        if not os.path.exists(path):
            self.get_logger().warn('IMU calibration file not found — using identity.')
            return np.eye(3)
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        key = 'imu_calibration_tip'
        if key not in data:
            self.get_logger().warn(f'Key "{key}" missing from IMU cal yaml — using identity.')
            return np.eye(3)
        return np.array(data[key], dtype=float)

    # * write the final R_mag_to_robot matrix to yaml
    def write_down_matrix(self, R: np.ndarray) -> None:
        local_dir         = os.path.expanduser('~/.ros/calibration')
        local_config_file = os.path.join(local_dir, 'mag_cal_tip.yaml')
        os.makedirs(local_dir, exist_ok=True)
        if not os.path.exists(local_config_file):
            pkg_share_path = get_package_share_directory('calibration')
            shutil.copyfile(
                src=os.path.join(pkg_share_path, 'config', 'mag_cal_tip.yaml'),
                dst=local_config_file)
        with open(file=local_config_file, mode='w') as file_handle:
            yaml.safe_dump(
                stream=file_handle,
                data={'magnetometer_calibration_tip': R.tolist()})
        self.get_logger().info('Successfully created calibration yaml.')


def main():
    rclpy.init()
    mynode = CalibrateMagneticSensor()
    while mynode.done is not True:
        rclpy.spin_once(mynode)
    mynode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()