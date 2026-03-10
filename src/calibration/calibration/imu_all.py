#!/usr/bin/env python3

"""
* This script gets the neutral values of all imu sensors (both topics raw and processed). For this the robot rotates along 
* defined axis, storing its linear acceleration and gravity data in a local imu_cal_all.yaml for later use.
--------------------------------------------------------------------------------------------------------
First the robot drives to an upright position. After the system stops swinging the sensors data gets
measured over a short period of time to determine the mean value thats set as calibration.

Second the robot tilts around the x-axis. The changed vector together with the prior one will 
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

class CalibrateimuSensor(Node):

    # * function called on creation
    def __init__(self):
        # "it insists upon itself"
        super().__init__('imu_sensor_calibration_all')

        # * ROS related init
        # micro ros cant guarantee loss free data transmission so this is to match the expected QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # create subscribers to get the vector from the imu sensor
        self.subscription_acc_1  = self.create_subscription(Accel, '/teensy_hub/imu_data_1', self.callback_imu_acc_1, qos)
        self.subscription_acc_2  = self.create_subscription(Accel, '/teensy_hub/imu_data_2', self.callback_imu_acc_2, qos)
        self.subscription_acc_3  = self.create_subscription(Accel, '/teensy_hub/imu_data_3', self.callback_imu_acc_3, qos)
        self.subscription_acc_4  = self.create_subscription(Accel, '/teensy_hub/imu_data_4', self.callback_imu_acc_4, qos)
        # same with imu internal grav
        self.subscription_grav_1  = self.create_subscription(Vector3, '/teensy_hub/imu_gravity_data_1', self.callback_imu_grav_1, qos)
        self.subscription_grav_2  = self.create_subscription(Vector3, '/teensy_hub/imu_gravity_data_2', self.callback_imu_grav_2, qos)
        self.subscription_grav_3  = self.create_subscription(Vector3, '/teensy_hub/imu_gravity_data_3', self.callback_imu_grav_3, qos)
        self.subscription_grav_4  = self.create_subscription(Vector3, '/teensy_hub/imu_gravity_data_4', self.callback_imu_grav_4, qos)
        # link into the servo topic to command the robots actuators
        self.servo_pub = self.create_publisher(ServoCommands, '/teensy_hub/servo_pos', 10)
        # create a timer to oversee progress
        self.planner = self.create_timer(0.1, self.planner_callback)

        # * Parameter from pose_cal.yaml
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
        self.sampling_time = 2
        # boolean to decide whether to catch sensor data
        self.sampling = False
        # dict with lists to store sensor data in
        self.samples = {
            name: { 
                key: [] for key in range(1, 5)
            } for name in ('acc', 'grav') }
        # boolean to exit cleanly
        self.done = False
        # struct for storing the calibrated axis values
        self.axis = {
            name: { 
                key: {
                    'x-axis': {'x': 1.0, 'y': 0.0, 'z': 0.0},
                    'y-axis': {'x': 0.0, 'y': 1.0, 'z': 0.0},
                    'z-axis': {'x': 0.0, 'y': 0.0, 'z': 1.0},
                } for key in range(1, 5)
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
            self.get_logger().info('Stopped collecting data.', once = True)
            # after sampling is done for the rotated position define the new xaxis
            self.save_xaxis()
            # now that two axis have been made out the last one can be calculated from those
            self.save_yaxis()
            # with all axis figured out a transformation matrix can be made
            # print it out
            for key in range(1, 5):
                matrix_acc = self.get_rot_matrix('acc', key)
                self.get_logger().info(f'Matrix Acc {key}:\n' + pprint.pformat(matrix_acc))
                matrix_grav = self.get_rot_matrix('grav', key)
                self.get_logger().info(f'Matrix Grav {key}:\n' + pprint.pformat(matrix_grav))
            # calls function to handle saving to yaml
            self.write_down_axis()
            # my job here is done
            self.done = True
    
    # * function to save new zaxis
    def save_zaxis(self) -> None:
        # go through each sensor
        for key in range(1,5):
            # get the mean value of the samples
            v_acc, v_grav = self.evaluate_samples(key)
            # those vectors point the wrong direction since gravity vector shows down
            v_acc_neg = Vector3(x=-v_acc.x, y=-v_acc.y, z=-v_acc.z)
            v_grav_neg = Vector3(x=-v_grav.x, y=-v_grav.y, z=-v_grav.z)
            # put those values into the new defined z-axis
            self.set_axis_vector(v_acc_neg, 'acc', key, 'z-axis')
            self.set_axis_vector(v_grav_neg, 'grav', key, 'z-axis')
            # clear out all data so next sample process can start
            self.samples['acc'][key].clear()
            self.samples['grav'][key].clear()

    # * function to save new yaxis
    def save_yaxis(self) -> None:
        # go through each sensor
        for key in range(1,5):
            # get the saved z-axis
            z_acc, z_grav = self.get_axis_vector(key, 'z-axis')
            # get the saved x-axis
            x_acc, x_grav = self.get_axis_vector(key, 'x-axis')
            # calculate the cross product
            y_acc = self.cross_product(z_acc, x_acc)
            y_grav = self.cross_product(z_grav, x_grav)
            # save this new vector as the new x-axis since the sensors got rotated around it
            self.set_axis_vector(y_acc, 'acc', key, 'y-axis')
            self.set_axis_vector(y_grav, 'grav', key, 'y-axis')

    # * function to save new xaxis
    def save_xaxis(self) -> None:
        for key in range(1, 5):
            # get the mean value of the new samples
            v2_acc, v2_grav = self.evaluate_samples(key)
            # get the last saved measurement (which was the negative z-axis)
            z_acc, z_grav = self.get_axis_vector(key, 'z-axis')
            # calculate the normal vector of the plane spanned by this new sampled axis and the z-axis
            x_acc  = self.cross_product(z_acc, v2_acc)
            x_grav = self.cross_product(z_grav, v2_grav)
            # this will be our newly defined y-axis
            self.set_axis_vector(x_acc, 'acc', key, 'x-axis')
            self.set_axis_vector(x_grav, 'grav', key, 'x-axis')
            # clean up crew
            self.samples['acc'][key].clear()
            self.samples['grav'][key].clear()

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
    def callback_imu_grav_1(self, msg: Vector3) -> None: 
        if self.sampling: self.samples['grav'][1].append([msg.x, msg.y, msg.z])
    def callback_imu_grav_2(self, msg: Vector3) -> None:
        if self.sampling: self.samples['grav'][2].append([msg.x, msg.y, msg.z])
    def callback_imu_grav_3(self, msg: Vector3) -> None:
        if self.sampling: self.samples['grav'][3].append([msg.x, msg.y, msg.z])
    def callback_imu_grav_4(self, msg: Vector3) -> None:
        if self.sampling: self.samples['grav'][4].append([msg.x, msg.y, msg.z])
    # same goes for the acceleration data, just in another topic and struct
    def callback_imu_acc_1(self, msg: Accel) -> None:
        if self.sampling: self.samples['acc'][1].append([msg.linear.x, msg.linear.y, msg.linear.z])
    def callback_imu_acc_2(self, msg: Accel) -> None:
        if self.sampling: self.samples['acc'][2].append([msg.linear.x, msg.linear.y, msg.linear.z])
    def callback_imu_acc_3(self, msg: Accel) -> None:
        if self.sampling: self.samples['acc'][3].append([msg.linear.x, msg.linear.y, msg.linear.z])
    def callback_imu_acc_4(self, msg: Accel) -> None:
        if self.sampling: self.samples['acc'][4].append([msg.linear.x, msg.linear.y, msg.linear.z])

    # * function to evaluate collected samples
    def evaluate_samples(self, key:int) -> tuple[Vector3, Vector3]:
        # convert to numpy arrays for faster computing
        v_acc, v_grav = np.array(self.samples['acc'][key]), np.array(self.samples['grav'][key])
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
    def get_axis_vector(self, key:int, axis:str) -> tuple[Vector3, Vector3]:
        return (Vector3(x=self.axis['acc'][key][axis]['x'], y=self.axis['acc'][key][axis]['y'], z=self.axis['acc'][key][axis]['z']),
                Vector3(x=self.axis['grav'][key][axis]['x'], y=self.axis['grav'][key][axis]['y'], z=self.axis['grav'][key][axis]['z']) ) 
    
    # * function to save Vector3 into self.axis
    def set_axis_vector(self, vector: Vector3, topic: str, key: int, axis: str) -> None:
        self.axis[topic][key][axis] = {'x': vector.x, 'y': vector.y, 'z': vector.z}

    # * function to get rotational matrix
    def get_rot_matrix(self, topic: str, key:int) ->  list[list[float]]:
        return [[self.axis[topic][key]['x-axis']['x'], self.axis[topic][key]['x-axis']['y'], self.axis[topic][key]['x-axis']['z']],
                [self.axis[topic][key]['y-axis']['x'], self.axis[topic][key]['y-axis']['y'], self.axis[topic][key]['y-axis']['z']],
                [self.axis[topic][key]['z-axis']['x'], self.axis[topic][key]['z-axis']['y'], self.axis[topic][key]['z-axis']['z']]]

    # * function to write results in yaml file
    # calibration data needs to survive rebuilds so its stored in  a local dir ~/.ros/...
    # this enables reloading calibration from a prior build (less annoying)    
    def write_down_axis(self) -> None:
        # path to the needed config file (expanduser since python cant handle ~)
        local_dir = os.path.expanduser('~/.ros/calibration')
        local_config_file = os.path.join(local_dir, 'imu_cal_all.yaml')
        # create a local dir (if it hasnt been made)
        os.makedirs(local_dir, exist_ok=True)
        # if theres no file
        if not os.path.exists(local_config_file):
            # find the default yaml path
            pkg_share_path = get_package_share_directory("calibration")
            # copy the default yaml file into the desired location 
            shutil.copyfile(
                src = os.path.join(pkg_share_path, 'config', 'imu_cal_all.yaml'),
                dst = local_config_file)
        # now that the file exists any way open it in write mode
        with open(file = local_config_file, mode = 'w') as file_handle:
            # dump the struct of axis in it (matches the yamls structure)
            yaml.safe_dump(
                stream = file_handle,
                data = { # dont forget to add the namespace
                    'imu_acc': {
                        key: self.get_rot_matrix('acc', key) for key in range(1, 5)
                    },
                    'imu_grav': {
                        key: self.get_rot_matrix('grav', key) for key in range(1, 5)
                    }, } )
        self.get_logger().info('Succesfully created calibration yaml.')

def main():
    rclpy.init()
    mynode = CalibrateimuSensor()
    while mynode.done is not True:
        rclpy.spin_once(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()