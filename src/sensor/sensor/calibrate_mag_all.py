#!/usr/bin/env python3

"""
* This script gets the neutral values of ALL the magnetic sensors. For this the robot rotates along 
* defined axis, storing its magnetic sensor data in a shared file for later programs to use
--------------------------------------------------------------------------------------------------------
First the robot drives to an upright position. After the system stops swinging the sensors data gets
measured over a short period of time to determine the mean value thats set as calibration.

Second the robot tilts around the x-axis. The changed magnetic vector together with the prior one will 
span a plane. From this plane the normal vector is the calibrated x-axis.

Lastly the cross product of the calibrated x and z axis will be declared as the calibrated y-axis.
"""

import numpy as np                                                  # useful for mathematics
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from rclpy.parameter import Parameter                               # be able to use params
from rcl_interfaces.msg import ParameterDescriptor                  # describe them if i forget or theyre named too short
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # specifics of transmission protocol for ros topics
from soro_msgs.msg import ServoCommands                             # datatype for publishing servos 
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
import yaml                                                         # access to yaml file
import os, shutil                                                   # operatin system, needed to write into another file
from ament_index_python.packages import get_package_share_directory # gets path to this package indepent from each pc
import pprint                                                       # pretty printing, better visuals for list

class CalibrateMagneticSensors(Node):
    
    # * function called on creation
    def __init__(self):
        # "it insists upon itself"
        super().__init__('hall_sensor_calibration_all')
        # * Parameter from pose_cal.yaml
        # read in the given params from the YAML file
        self.declare_parameters(
            namespace='',
            parameters=[ 
                ('upright', Parameter.Type.INTEGER_ARRAY, ParameterDescriptor(description='Servo values for an upright pose along the z-axis.')),
                ('rotated', Parameter.Type.INTEGER_ARRAY, ParameterDescriptor(description='Servo values for a slight rotation around the x-axis.')),])
        # * Parameter from this node
        # how often should the timer be called in one second
        self.hz             = 60
        # how long to wait until the robot is stationary (no swinging) in seconds
        self.wait           = 4
        # how long it takes to take the new position
        self.drive          = 1
        # how long one would like to sample
        self.sampling_time  = 10
        # boolean to decide whether to catch sensor data
        self.sampling       = False
        # number of sensors
        self.number         = 4
        # dict with identifiers for each sensor, storing data seperatly
        self.samples        = {i:[] for i in range(1, self.number+1)}
        # boolean to exit cleanly
        self.done           = False
        # struct for storing the calibrated axis values
        self.axis           = {
            i: {
            'x-axis': {'x': 1.0, 'y': 0.0, 'z': 0.0},
            'y-axis': {'x': 0.0, 'y': 1.0, 'z': 0.0},
            'z-axis': {'x': 0.0, 'y': 0.0, 'z': 1.0},
            }  for i in range(1, self.number+1) }
        # * ROS related init
        # micro ros cant guarantee loss free data transmission so this is to match the expected QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # create a list for the subscripers
        self.subscribers = []
        for key in range(1, self.number+1):
            subscription_n  = self.create_subscription(
                msg_type    = Vector3,
                topic       = f'/teensy_hub/mag_data_{key}',
                callback    = lambda msg, k=key: self.mag_callback(msg, k),
                qos_profile = qos )
            # appending to a list is fine, it will not be needed anymore so keys/index mixup is fine here
            self.subscribers.append(subscription_n)
        # link into the servo topic to command the robots actuators
        self.servo_pub    = self.create_publisher(ServoCommands, '/teensy_hub/servo_pos', 10)
        # create a timer to oversee progress, mark down the starting time
        self.planner      = self.create_timer(1/self.hz, self.planner_callback)
        self.start_time   = self.planner.clock.now().nanoseconds

    # * callback function of the planning timer
    def planner_callback(self) -> None:
        # simply skip if the program is done and the node is somehow still spinning
        if self.done is True:   return
        # first calculate how long the porgram has been running in seconds
        delta_t = (self.planner.clock.now().nanoseconds - self.start_time)/1e9
        # then differentiate at which time period what needs to be done:
        if delta_t < self.drive + self.wait:
            # drive to the upright position and hold it until stationary
            self.command_servos('upright')
            # just some clean output to know the progress
            self.get_logger().info('Driving to an upright position, to define the z-axis', once = True)
        elif delta_t < self.drive+self.wait+self.sampling_time:
            # changed boolean to accept incoming data
            self.sampling = True
            # some visual feedback
            self.get_logger().info('Started collecting data.', once = True)
        elif delta_t < 2*(self.drive+self.wait)+self.sampling_time:
            if self.sampling is True:
                # stop accepting sensor data
                self.sampling = False
                # give some visual feedback
                self.get_logger().info('Stopped collecting data.', once = True)
                # after sampling is done for the upright position define the new zaxis
                self.save_zaxis()
            # drive to the around x-axis tilted position and hold until stationary
            self.command_servos('rotated')
            # give some info on whats next
            self.get_logger().info('Driving to a new position. (rotating around x-axis)', once = True)
            # if statement to only do evaluation once
        elif delta_t < 2*(self.drive+self.wait+self.sampling_time):
            # change boolean to accept incoming data
            self.sampling = True
            # some visual feedback
            self.get_logger().info('Started collecting data again.', once = True)
        elif delta_t < 3*(self.drive+self.wait) + 2*self.sampling_time:
            # turn off sampling
            self.sampling = False
            # print out some information
            self.get_logger().info('Stopped collecting data.', once = True)
            # after sampling is done for the rotated position define the new xaxis
            self.save_xaxes()
            # now that two axis have been made out the last one can be calculated from those
            self.save_yaxes()
            # from these axis get the transformation matrices and save them in a local file
            transformation_matrices = {}
            # go through each sensor
            for key in range(1,self.number+1):
                # calculate rotational matrix
                matrix_n = self.get_rot_matrix(key)
                # add it to the struct of matrices
                transformation_matrices[key] = matrix_n
                # print it out
                self.get_logger().info(f'Matrix {key}:\n' + pprint.pformat(matrix_n))
            # write the matrices into the yaml
            self.write_down_axis(transformation_matrices)
            # visual confirmation that the program is completed
            self.get_logger().info('Done.', once = True)
            # my job here is done
            self.done = True
    
    # * function to move the robot
    def command_servos(self, pose_name: str) -> None:
        # create an message for servo commands
        servo_msg = ServoCommands()
        # load the integer array from the corresponding parameter 
        servo_msg.servo_micros = self.get_parameter(pose_name).get_parameter_value().integer_array_value
        # publish the commands to move the robot
        self.servo_pub.publish(servo_msg)
    
    # * callback function of the sensors
    def mag_callback(self, msg: Vector3, key: int) -> None: 
        # only collect data when needed else skip
        if self.sampling: self.samples[key].append([msg.x, msg.y, msg.z])

    # * function to save new xaxis for each sensor
    def save_xaxes(self) -> None:
        # go through each sensor
        for key in range(1, self.number+1):
            # grab the vector of the previous defined z-axis
            v1 = self.get_axis_vector(key, 'z-axis')
            # calculate mean vectors from samples
            v2 = self.evaluate_samples(key)
            # calculate the normal vector of the plane spanned by this new sampled axis and the z-axis
            v1x2  = self.cross_product(v1, v2)
            # save this new vector as the new x-axis since the sensors got rotated around it
            self.set_axis_vector(v1x2, key, 'x-axis')
            # cleanup crew
            self.samples[key].clear()
    
    # * function to save new yaxis for each sensor
    def save_yaxes(self) -> None:
        # go through each sensor
        for key in range(1, self.number+1):
            # grab the vector of the previous defined z-axis
            v1 = self.get_axis_vector(key, 'z-axis')
            # grab the vector of the previous defined x-axis
            v2 = self.get_axis_vector(key, 'x-axis')
            # cross product will give the new y-axis
            v1x2 = self.cross_product(v1, v2)
            # save this as the new y-axis
            self.set_axis_vector(v1x2, key, 'y-axis')
    
    # * function to save new zaxis for each sensor
    def save_zaxis(self) -> None:
        # go through each sensor
        for key in range(1, self.number+1):
            # calculate mean vectors from samples 
            v = self.evaluate_samples(key)
            # save this vector as the new zaxis
            self.set_axis_vector(v, key, 'z-axis')
            # empty the samples list after all data has been proccessed
            self.samples[key].clear()
    
    # * function to evaluate samples
    def evaluate_samples(self, key: int) -> Vector3:
        # convert to numpy arrays for faster computing
        vectors = np.array(self.samples[key])
        # calculate the mean of all the array (axis=0 means numpy takes rows as direction, which is the case for a list)
        mean = np.mean(vectors, axis=0)
        # normalize it
        mean = mean/np.linalg.norm(mean)
        # return a vector with float datatypes instead of a numpy array
        return Vector3(x=mean[0].item(), y=mean[1].item(), z=mean[2].item())

    # * function to calculate cross vector
    def cross_product(self, vector1: Vector3, vector2: Vector3) -> Vector3:
        # convert to numpy for easier calculation
        v1 = np.array([vector1.x, vector1.y, vector1.z])
        v2 = np.array([vector2.x, vector2.y, vector2.z])
        # calculate crossproduct
        v1x2 = np.cross(v1,v2)
        # because of the unkown angle between v1 and v2 the cross product needs to be normalized
        v3 = v1x2/np.linalg.norm(v1x2)
        # return a vector with float datatypes instead of a numpy array
        return Vector3(x=v3[0].item(), y=v3[1].item(), z=v3[2].item())
    
    # * function to convert self.axis to Vector3 type
    def get_axis_vector(self, key:int, name:str) -> Vector3:
        return Vector3(x=self.axis[key][name]['x'], y=self.axis[key][name]['y'], z=self.axis[key][name]['z'])
    
    # * function to save Vector3 into self.axis
    def set_axis_vector(self, vector: Vector3, key: int, name: str) -> None:
        self.axis[key][name] = {'x': vector.x, 'y': vector.y, 'z': vector.z}

    # * function to get rotational matrix
    def get_rot_matrix(self, key: int) ->  list:
        return [[self.axis[key]['x-axis']['x'], self.axis[key]['y-axis']['x'], self.axis[key]['z-axis']['x']],
                [self.axis[key]['x-axis']['y'], self.axis[key]['y-axis']['y'], self.axis[key]['z-axis']['y']],
                [self.axis[key]['x-axis']['z'], self.axis[key]['y-axis']['z'], self.axis[key]['z-axis']['z']]]

    # * function to write results in yaml file
    # calibration data needs to survive rebuilds so its stored in  a local dir ~/.ros/...
    # this enables reloading calibration from a prior build (less annoying)    
    def write_down_axis(self, matrices: dict) -> None:
        # path to the local config file and its dir (expanduser since python cant handle ~)
        local_dir = os.path.expanduser('~/.ros/sensor/calibration')
        local_calib_file = os.path.join(local_dir, 'mag_cal_all.yaml')
        # create a local dir if it hasnt been made
        os.makedirs(local_dir, exist_ok=True)
        # if theres no file
        if not os.path.exists(local_calib_file):
            # find the default yaml path
            pkg_share_path = get_package_share_directory("sensor")
            # copy the default yaml file into the desired location 
            shutil.copyfile(
                src = os.path.join(pkg_share_path, 'config', 'mag_cal_all.yaml'),
                dst = local_calib_file)
        # now that the file exists any way open it in write mode
        with open(file = local_calib_file, mode = 'w') as file_handle:
            # dump the struct of axis in it (matches the yamls structure)
            yaml.safe_dump(
                stream = file_handle,
                data = { # dont forget to add the namespace
                    'magnetometer_calibration_all': {
                        # matrices is a list using idx not keys
                        key: matrices[key] for key in range(1, self.number+1)
                    }},)
        self.get_logger().info('Succesfully created calibration yaml.')

def main():
    rclpy.init()
    mynode = CalibrateMagneticSensors()
    while mynode.done is not True:
        rclpy.spin_once(mynode)
    rclpy.shutdown()
    mynode.destroy_node()

if __name__ == '__main__':
    main()