#!/usr/bin/env python3

"""
* This script gets the neutral values of the magnetic sensor at the tip. For this the robot rotates along 
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
from soro_msgs.msg import ServoCommands                             # datatype for publishing servos 
from geometry_msgs.msg import Vector3                               # datatype for three dimensional vectors (used in subscribing mag_sensor)
import yaml                                                         # access to yaml file
import os, shutil                                                   # operatin system, needed to write into another file
from ament_index_python.packages import get_package_share_directory # gets path to this package indepent from each pc
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
        # create a subscriber to get the vector from the magnetic sensor
        self.subscription = self.create_subscription(
            msg_type    = Vector3,
            topic       = f'/teensy_hub/mag_data_{self.tip}',
            callback    = self.mag_callback,
            qos_profile = qos
        )
        # link into the servo topic to command the robots actuators
        self.servo_pub = self.create_publisher(
            msg_type    = ServoCommands,
            topic       = '/teensy_hub/servo_pos',
            qos_profile = 10)
        # create a timer to oversee progress
        self.planner = self.create_timer(
            timer_period_sec    = 0.1,
            callback            = self.planner_callback,)
        # mark down the starting point of the timer
        self.start_time = self.planner.clock.now().nanoseconds
        # * Parameter from pose_cal.yaml
        # read in the given params from the YAML file
        self.declare_parameters(
            namespace='',
            parameters=[ 
                ('upright', Parameter.Type.INTEGER_ARRAY, ParameterDescriptor(description='Servo values for an upright pose along the z-axis.')),
                ('rotated', Parameter.Type.INTEGER_ARRAY, ParameterDescriptor(description='Servo values for a slight rotation around the x-axis.')),])
        # * Parameter for this node
        # how long to wait until the robot is stationary (no swinging) in seconds
        self.wait = 3
        # how long it takes to take the new position
        self.drive = 1
        # how long one would like to sample
        self.sampling_time = 2
        # boolean to decide whether to catch sensor data
        self.sampling = False
        # list to store sensor data in
        self.samples = []
        # boolean to exit cleanly
        self.done = False
        # struct for storing the calibrated axis values
        self.axis = {
            'x-axis': {'x': 1.0, 'y': 0.0, 'z': 0.0},
            'y-axis': {'x': 0.0, 'y': 1.0, 'z': 0.0},
            'z-axis': {'x': 0.0, 'y': 0.0, 'z': 1.0},}

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
            self.get_logger().info('Driving to an upright position.', once = True)
        elif delta_t < self.drive+self.wait+self.sampling_time:
            # changed boolean to accept incoming data
            self.sampling = True
            # some visual feedback
            self.get_logger().info('Started collecting data.', once = True)
        elif delta_t < 2*(self.drive+self.wait)+self.sampling_time:
            # if statement to only do evaluation once
            if self.sampling is True:
                # stop accepting sensor data
                self.sampling = False
                # give some visual feedback
                self.get_logger().info('Stopped collecting data.', once = True)
                # evaluate data
                self.save_zaxis()
            # drive to the around x-axis tilted position and hold until stationary
            self.command_servos('rotated')
            # give some info on whats next
            self.get_logger().info('Driving to a new position. (rotating around x-axis)', once = True)
        elif delta_t < 2*(self.drive+self.wait+self.sampling_time):
            # changed boolean to accept incoming data
            self.sampling = True
            # some visual feedback
            self.get_logger().info('Started collecting data again.', once = True)
        elif delta_t > 2*(self.drive+self.wait+self.sampling_time):
            # turn off accepting new data
            self.sampling = False
            # print out some information
            self.get_logger().info('Stopped collecting data', once = True)
            # after sampling is done for the rotated position define the new xaxis
            self.save_xaxis()
            # now that two axis have been made out the last one can be calculated from those
            self.save_yaxis()
            # with all axis figured out a transformation matrix can be made
            matrix = self.get_rot_matrix()
            # print it out
            self.get_logger().info('Matrix:\n' + pprint.pformat(matrix))
            # calls function to handle saving to yaml
            self.write_down_axis()
            # my job here is done
            self.done = True
    
    # * function to save new zaxis
    def save_zaxis(self) -> None:
        # get the mean value of the samples
        v = self.evaluate_samples()
        # put those values into the new defined z-axis
        self.set_axis_vector(v, 'z-axis')
        # clear out all data so next sample process can start
        self.samples.clear()

    # * function to save new yaxis
    def save_yaxis(self) -> None:
        # get the saved z-axis
        v1 = self.get_axis_vector('z-axis')
        # get the saved x-axis
        v2 = self.get_axis_vector('x-axis')
        # calculate the cross product
        v1x2 = self.cross_product(v1, v2)
        # save this new vector as the new x-axis since the sensors got rotated around it
        self.set_axis_vector(v1x2, 'y-axis')

    # * function to save new xaxis
    def save_xaxis(self) -> None:
        v1 = self.get_axis_vector('z-axis')
        # get the mean value
        v2 = self.evaluate_samples()
        # calculate the normal vector of the plane spanned by this new sampled axis and the z-axis
        v1x2  = self.cross_product(v1, v2)
        # this will be our newly defined y-axis
        self.set_axis_vector(v1x2, 'x-axis')
        # clean up crew
        self.samples.clear()

    # * function to move the robot
    def command_servos(self, pose_name: str) -> None:
        # create an message for servo commands
        servo_msg = ServoCommands()
        # load the integer array from the corresponding parameter 
        servo_msg.servo_micros = self.get_parameter(pose_name).get_parameter_value().integer_array_value
        # publish the commands to move the robot
        self.servo_pub.publish(servo_msg)
    
    # * callback function of the sensor
    # while the robot is moving (sampling is False) ignore incoming data
    def mag_callback(self, msg) -> None:
        if self.sampling: self.samples.append([msg.x, msg.y, msg.z])
    
    # * function to evaluate collected samples
    def evaluate_samples(self) -> Vector3:
        # convert to numpy arrays for faster computing
        vectors = np.array(self.samples)
        # calculate the mean of all the array (axis=0 means numpy takes rows as direction, which is for us all data)
        mean = np.mean(vectors, axis=0)
        # normalize it
        mean = mean/np.linalg.norm(mean)
        # return a vector with float datatypes instead of a numpy array
        return Vector3(x=mean[0].item(), y=mean[1].item(), z= mean[2].item())

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
    def get_axis_vector(self, name:str) -> Vector3:
        return Vector3(x=self.axis[name]['x'], y=self.axis[name]['y'], z=self.axis[name]['z'])
    
    # * function to save Vector3 into self.axis
    def set_axis_vector(self, vector: Vector3, name: str) -> None:
        self.axis[name] = {'x': vector.x, 'y': vector.y, 'z': vector.z}

    # * function to get rotational matrix
    def get_rot_matrix(self) -> list[list[float]]:
        return [[self.axis['x-axis']['x'], self.axis['x-axis']['y'], self.axis['x-axis']['z']],
                [self.axis['y-axis']['x'], self.axis['y-axis']['y'], self.axis['y-axis']['z']],
                [self.axis['z-axis']['x'], self.axis['z-axis']['y'], self.axis['z-axis']['z']]]

    # * function to write results in yaml file
    # calibration data needs to survive rebuilds so its stored in  a local dir ~/.ros/...
    # this enables reloading calibration from a prior build (less annoying)    
    def write_down_axis(self) -> None:
        # path to the needed config file (expanduser since python cant handle ~)
        local_dir = os.path.expanduser('~/.ros/calibration')
        local_config_file = os.path.join(local_dir, 'mag_cal_tip.yaml')
        # create a local dir (if it hasnt been made)
        os.makedirs(local_dir, exist_ok=True)
        # if theres no file
        if not os.path.exists(local_config_file):
            # find the default yaml path
            pkg_share_path = get_package_share_directory('sensor')
            # copy the default yaml file into the desired location 
            shutil.copyfile(
                src = os.path.join(pkg_share_path, 'config', 'mag_cal_tip.yaml'),
                dst = local_config_file)
        # now that the file exists any way open it in write mode
        with open(file = local_config_file, mode = 'w') as file_handle:
            # dump the struct of axis in it (matches the yamls structure)
            yaml.safe_dump(
                stream = file_handle,
                data = { # dont forget to add the namespace
                    'magnetometer_calibration_tip': self.get_rot_matrix()},)
        self.get_logger().info('Succesfully created calibration yaml.')

def main():
    rclpy.init()
    mynode = CalibrateMagneticSensor()
    while mynode.done is not True:
        rclpy.spin_once(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()