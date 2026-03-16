#!/usr/bin/env python3

"""
* This script gets the neutral values of the imu sensor at the tip. For this the robot rotates along 
* defined axis, storing its linear acceleration and gravity data in a local imu_cal_tip.yaml file for later use.
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
from geometry_msgs.msg import Accel                                 # datatype used by imu raw (accel) topic
from geometry_msgs.msg import Vector3                               # datatype used by imu gravity topic
import yaml                                                         # access to yaml file
import os, shutil                                                   # operatin system, needed to write into another file
from ament_index_python.packages import get_package_share_directory # gets path to this package indepent from each pc
import pprint                                                       # pretty printing, better visuals for list

class Calibrate_imu_sensor(Node):

    # * function called on creation
    def __init__(self):
        # "it insists upon itself"
        super().__init__('imu_sensor_calibration_tip')
        # kept constant, identifier for the sensor at the tip of the robot (for one robot it was one for the other 4 so its kept as a constant)
        self.TIP = 1 
        # * ROS related init
        # match microROS QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # create a subscriber to get the vector from the imu sensor
        # once for linear acceleration vector
        self.subscription_acc = self.create_subscription(
            msg_type    = Accel,
            topic       = f'/teensy_hub/imu_data_{self.TIP}',
            callback    = self.imu_callback_acc,
            qos_profile = qos)
        # once for gravity vector
        self.subscription_grav = self.create_subscription(
            msg_type    = Vector3,
            topic       = f'/teensy_hub/imu_gravity_data_{self.TIP}',
            callback    = self.imu_callback_grav,
            qos_profile = qos)
        # link into the servo topic to command the robots actuators
        self.servo_pub = self.create_publisher(
            msg_type    = ServoCommands,
            topic       = '/teensy_hub/servo_pos',
            qos_profile = 10)
        # create a timer to oversee progress
        self.planner = self.create_timer(
            timer_period_sec    = 0.1,
            callback            = self.planner_callback,)
        # * Parameter from pose_cal_4.yaml
        # read in the given params from the YAML file
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rotated', Parameter.Type.INTEGER_ARRAY, ParameterDescriptor(description='Servo values for a slight rotation around the x-axis.')),
                ('upright', Parameter.Type.INTEGER_ARRAY, ParameterDescriptor(description='Servo values for an upright pose along the z-axis.')),])
        # * Parameter for this node
        # mark down the starting point of the timer
        self.start_time = self.planner.clock.now().nanoseconds
        # how long to wait until the robot is stationary (no swinging) in seconds
        self.wait = 3
        # how long it takes to take the new position
        self.drive = 1
        # how long one would like to sample
        self.sampling_time = 5
        # boolean to decide whether to catch sensor data
        self.sampling = False
        # list to store sensor data in
        self.samples = {'acc':[], 'grav':[]}
        # boolean to exit cleanly
        self.done = False
        # struct for storing the calibrated axis values
        self.axis = {
            name: {
            'x-axis': {'x': 1.0, 'y': 0.0, 'z': 0.0},
            'y-axis': {'x': 0.0, 'y': 1.0, 'z': 0.0},
            'z-axis': {'x': 0.0, 'y': 0.0, 'z': 1.0},
            } for name in ('acc', 'grav') }

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
            matrix_acc = self.get_rot_matrix('acc')
            matrix_grav = self.get_rot_matrix('grav')
            # print it out
            self.get_logger().info('Matrix Acc:\n' + pprint.pformat(matrix_acc))
            self.get_logger().info('Matrix Grav:\n' + pprint.pformat(matrix_grav))
            # calls function to handle saving to yaml
            self.write_down_axis()
            # my job here is done
            self.done = True
    
    # * function to save new zaxis
    def save_zaxis(self) -> None:
        # get the mean value of the samples
        v_acc, v_grav = self.evaluate_samples()
        # put those values into the new defined z-axis
        self.set_axis_vector(v_acc, 'acc', 'z-axis')
        self.set_axis_vector(v_grav, 'grav', 'z-axis')
        # clear out all data so next sample process can start
        self.samples['acc'].clear()
        self.samples['grav'].clear()

    # * function to save new yaxis
    def save_yaxis(self) -> None:
        # get the saved z-axis
        z_acc, z_grav = self.get_axis_vector('z-axis')
        # get the saved x-axis
        x_acc, x_grav = self.get_axis_vector('x-axis')
        # calculate the cross product
        y_acc = self.cross_product(z_acc, x_acc)
        y_grav = self.cross_product(z_grav, x_grav)
        # save this new vector as the new x-axis since the sensors got rotated around it
        self.set_axis_vector(y_acc, 'acc', 'y-axis')
        self.set_axis_vector(y_grav, 'grav', 'y-axis')
        # no need to clear samples cause none were taken when this function is called

    # * function to save new xaxis
    def save_xaxis(self) -> None:
        # get the mean value of the second measurement
        v2_acc, v2_grav = self.evaluate_samples()
        # get the last saved measurement (which was the negative z-axis)
        z_acc, z_grav = self.get_axis_vector('z-axis')
        # calculate the normal vector of the plane spanned by this new sampled axis and the z-axis
        x_acc  = self.cross_product(z_acc, v2_acc)
        x_grav = self.cross_product(z_grav, v2_grav)
        # this will be our newly defined y-axis
        self.set_axis_vector(x_acc, 'acc', 'x-axis')
        self.set_axis_vector(x_grav, 'grav', 'x-axis')
        # clean up crew
        self.samples['acc'].clear()
        self.samples['grav'].clear()

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
    def imu_callback_grav(self, msg: Vector3) -> None:
        if self.sampling: self.samples['grav'].append([msg.x, msg.y, msg.z])
    def imu_callback_acc(self, msg:Accel) -> None:
        if self.sampling: self.samples['acc'].append([msg.linear.x, msg.linear.y, msg.linear.z])
    # Accel Class has two Vector3, linear and angular. For static assumption linear is enough 

    # * function to evaluate collected samples
    def evaluate_samples(self) -> tuple[Vector3, Vector3]:
        # convert to numpy arrays for faster computing
        v_acc, v_grav = np.array(self.samples['acc']), np.array(self.samples['grav'])
        # calculate the mean of all the array (axis=0 means numpy takes rows as direction, which is for us all data)
        mean_acc, mean_grav = np.mean(v_acc, axis=0), np.mean(v_grav, axis=0)
        # normalize the mean vectors
        mean_acc = mean_acc / np.linalg.norm(mean_acc)
        mean_grav = mean_grav / np.linalg.norm(mean_grav)
        # return a vector with float datatypes instead of a numpy array
        return ( Vector3(x = mean_acc[0].item(), y = mean_acc[1].item(), z = mean_acc[2].item()),
                 Vector3(x = mean_grav[0].item(), y = mean_grav[1].item(), z = mean_grav[2].item()) )

    # * function to calculate cross vector
    def cross_product(self, vector1: Vector3, vector2: Vector3) -> Vector3:
        # convert to numpy for easier calculation
        v1 = np.array([vector1.x, vector1.y, vector1.z])
        v2 = np.array([vector2.x, vector2.y, vector2.z])
        # calculate crossproduct
        v1x2 = np.cross(v1,v2)
        # because of the unkown angle between v1 and v2 the cross product needs to be normalized
        v3 = v1x2 / np.linalg.norm(v1x2)
        # return a vector with float datatypes instead of a numpy array
        return Vector3(x=v3[0].item(), y=v3[1].item(), z=v3[2].item())
    
    # * function to convert self.axis to Vector3 type
    def get_axis_vector(self, axis:str) -> tuple[Vector3, Vector3]:
        return (Vector3(x=self.axis['acc'][axis]['x'], y=self.axis['acc'][axis]['y'], z=self.axis['acc'][axis]['z']),
                Vector3(x=self.axis['grav'][axis]['x'], y=self.axis['grav'][axis]['y'], z=self.axis['grav'][axis]['z']) ) 
    
    # * function to save Vector3 into self.axis
    def set_axis_vector(self, vector: Vector3, topic: str, axis: str) -> None:
        self.axis[topic][axis] = {'x': vector.x, 'y': vector.y, 'z': vector.z}

    # * function to get rotational matrix
    def get_rot_matrix(self, topic: str) ->  list[list[float]]:
        return [[self.axis[topic]['x-axis']['x'], self.axis[topic]['x-axis']['y'], self.axis[topic]['x-axis']['z']],
                [self.axis[topic]['y-axis']['x'], self.axis[topic]['y-axis']['y'], self.axis[topic]['y-axis']['z']],
                [self.axis[topic]['z-axis']['x'], self.axis[topic]['z-axis']['y'], self.axis[topic]['z-axis']['z']]]

    # * function to write results in yaml file
    # calibration data needs to survive rebuilds so its stored in  a local dir ~/.ros/...
    # this enables reloading calibration from a prior build (less annoying)    
    def write_down_axis(self) -> None:
        # path to the needed config file (expanduser since python cant handle ~)
        local_dir = os.path.expanduser('~/.ros/calibration')
        local_config_file = os.path.join(local_dir, 'imu_cal_tip.yaml')
        # create a local dir (if it hasnt been made)
        os.makedirs(local_dir, exist_ok=True)
        # if theres no file
        if not os.path.exists(local_config_file):
            # find the default yaml path
            pkg_share_path = get_package_share_directory("calibration")
            # copy the default yaml file into the desired location 
            shutil.copyfile(
                src = os.path.join(pkg_share_path, 'config', 'imu_cal_tip.yaml'),
                dst = local_config_file)
        # now that the file exists any way open it in write mode
        with open(file = local_config_file, mode = 'w') as file_handle:
            # dump the struct of axis in it (matches the yamls structure)
            yaml.safe_dump(
                stream = file_handle,
                data = { # dont forget to add the namespace
                    'imu_acc': self.get_rot_matrix('acc'),
                    'imu_grav': self.get_rot_matrix('grav'),},)
        self.get_logger().info('Succesfully created calibration yaml.')

def main():
    rclpy.init()
    mynode = Calibrate_imu_sensor()
    while mynode.done is not True:
        rclpy.spin_once(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()