#! /usr/bin/env python3

"""
* This script implements a very simple PI controller.
* Because the project is assumed to be almost static (moving very slowly) the derivative/predictive part can be ignored.
* The errors are published by another node in the controller package to keep the code clean.
"""

import numpy as np                                                  # for math stuff
import rclpy                                                        # to be able to use ROS with python
from rclpy.node import Node                                         # ROS node creation
from std_msgs.msg import Float64MultiArray                          # datatype used for multiple floats
# following libs are all needed to have (dynamic) parameters, makes tuning the controller easier
from rcl_interfaces.msg import ParameterEvent
from rclpy.parameter_event_handler import ParameterEventHandler
from rcl_interfaces.msg import ParameterDescriptor              

class Controller(Node):

    # * function called on creation
    def __init__(self):
        # it insists upon itself
        super().__init__('controller')

        # * ROS related
        # subscribe to the error topic published by controller/publish_tendon_error.py
        self.subscription_error_bot = self.create_subscription(Float64MultiArray, '/pc/tendon_error_bot', lambda msg: self.callback_tendon_error(msg, 'bot'), 10)
        self.subscription_error_top = self.create_subscription(Float64MultiArray, '/pc/tendon_error_top', lambda msg: self.callback_tendon_error(msg, 'top'), 10)
        # publisher for the servo
        self.publisher_set_top = self.create_publisher(Float64MultiArray, '/pc/controller/output_top', 10)
        self.publisher_set_bot = self.create_publisher(Float64MultiArray, '/pc/controller/output_bot', 10)
        # to identify them
        self.publisher_select = { 'top': self.publisher_set_top, 'bot': self.publisher_set_bot }

        # * Parameter
        # for sanitys sake the gains of the controller are adjustable with rqt, allowing for quick testing
        self.declare_parameter('p_factor', 0.001, ParameterDescriptor(description = 'P Value for the controller.'))
        self.declare_parameter('i_factor', 0.0001, ParameterDescriptor(description = 'I Value for the controller.'))
        self.declare_parameter('windup_limit', 0.001, ParameterDescriptor(description = 'Limit for the I gain.'))
        # create an object monitoring for parameter changes
        self.param_handler = ParameterEventHandler(self)
        # give that monitoring object a callback function
        self.param_event_callback_handle = self.param_handler.add_parameter_event_callback(self.callback_param_handler)

        # * Variables for this node
        # when this node is initialized save the time
        init_time = self.get_clock().now().nanoseconds
        self.last_measurement_time = {'bot': init_time, 'top': init_time}
        # integral will be parsed over time so it needs to be accesible by this nodes function to keep track
        self.integral = {'bot': np.zeros(3), 'top': np.zeros(3)}

    # * callback function on parameter change
    def callback_param_handler(self, event: ParameterEvent) -> None:
        self.get_logger().info("Changed a parameter for this node.")
    
    # * callback function when receiving new information
    def callback_tendon_error(self, msg: Float64MultiArray, segment: str) -> None:
        # calculate the commands for the tendons
        output_a, output_b, output_c = self.compute_control_output(msg.data, segment)
        # fire it off
        self.publish_control_output(output_a, output_b, output_c, segment)
    
    # * function to calculate control output
    def compute_control_output(self, error: list[float], segment:str) -> tuple[float, float, float]: 
        # extract incoming tendon errors
        tendon_length_errors = np.asarray(error)
        # grab the factors for the controller
        k_p = self.get_parameter('p_factor').value
        k_i = self.get_parameter('i_factor').value
        # update the integral
        self.update_integral_of_error(error, segment)
        # make an array to apply the formula a step later
        integral = np.asarray( self.integral[segment] )
        # PI-Controller formula (directly as a vector)
        output = k_p * tendon_length_errors + k_i * integral
        # return the output
        return output[0].item(), output[1].item(), output[2].item()
    
    # * function to calculate the integral of the error
    def update_integral_of_error(self, error: list[float], segment: str):
        # extract incoming tendon errors
        tendon_length_errors = np.asarray(error)
        # also get the windup limit
        windup = self.get_parameter('windup_limit').value
        # calculate time that has passed (in seconds)
        dt = self.time_since_last_measurement(segment)
        # calculate Riemman integral square:
        integral_step = tendon_length_errors * dt
        # calculate the integral, while staying in windup boundaries
        self.integral[segment] = np.clip(self.integral[segment] + integral_step, [-windup]*3, [windup]*3)
        
    # * function to calculate time since last measurement
    def time_since_last_measurement(self, segment: str) -> float:
        # get current time
        current_time = self.get_clock().now().nanoseconds
        # calculate the time difference in seconds
        delta_t = (current_time - self.last_measurement_time[segment]) * 1e-9
        # "reset" the time to not integrate over too long of a time strip
        self.last_measurement_time[segment] = current_time
        # return the time difference
        return delta_t
    
    # * function to publish control output
    def publish_control_output(self, out1: float, out2: float, out3: float, segment:str) -> None:
        # prepare a message to publish
        msg = Float64MultiArray()
        # populate it with the results of the controller
        msg.data = [out1, out2, out3]
        # publish them to their respective publisher
        self.publisher_select[segment].publish(msg)
    
def main():
    rclpy.init()
    mynode = Controller()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()