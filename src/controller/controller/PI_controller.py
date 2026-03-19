#! /usr/bin/env python3

"""
* This script combines the tendon error calculation and PI controller into one node.
* It takes real tendon lengths (from IMU) and desired tendon lengths (from trajectory)
* and outputs absolute corrected tendon positions.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from rcl_interfaces.msg import ParameterEvent, ParameterDescriptor
from rclpy.parameter_event_handler import ParameterEventHandler

class Controller(Node):

    def __init__(self):
        # it insists upon itself
        super().__init__('controller')

        # * ROS related
        # subscribe to real tendon lengths from sensor/gen_coords -> kinematics/g2l
        self.subscription_real_bot     = self.create_subscription(Float64MultiArray, '/pc/tendon_lengths_bot',        lambda msg: self.callback_real(msg, 'bot'),    10)
        self.subscription_real_top     = self.create_subscription(Float64MultiArray, '/pc/tendon_lengths_top',        lambda msg: self.callback_real(msg, 'top'),    10)
        # subscribe to desired tendon lengths from controller/traj_coords -> kinematics/g2l
        self.subscription_desired_bot  = self.create_subscription(Float64MultiArray, '/pc/tendon_lengths_target_bot', lambda msg: self.callback_desired(msg, 'bot'), 10)
        self.subscription_desired_top  = self.create_subscription(Float64MultiArray, '/pc/tendon_lengths_target_top', lambda msg: self.callback_desired(msg, 'top'), 10)
        # publishers that will fire off absolute tendon lengths to plant
        self.publisher_select = {
            'top': self.create_publisher(Float64MultiArray, '/pc/controller/output_top', 10),
            'bot': self.create_publisher(Float64MultiArray, '/pc/controller/output_bot', 10),}
        # publish error diagnostics
        self.publisher_absolute_error  = self.create_publisher(Float64, '/pc/absolute_error', 10)
        self.publisher_average_error   = self.create_publisher(Float64, '/pc/average_error',  10)

        # * Geometric Parameters
        self.segment_length = self.declare_parameter('L_segment',    0.12,   ParameterDescriptor(description='Neutral segment length.')).value

        # * Parameters for controller
        self.declare_parameter('p_factor',     0.1,  ParameterDescriptor(description='P Value for the controller.'))
        self.declare_parameter('i_factor',     0.1,    ParameterDescriptor(description='I Value for the controller.'))
        self.declare_parameter('windup_limit', 0.5, ParameterDescriptor(description='Limit for the I gain.'))
        self.param_handler = ParameterEventHandler(self)
        self.param_event_callback_handle = self.param_handler.add_parameter_event_callback(self.callback_param_handler)

        # * Variables
        # placeholder lengths - start at neutral
        self.lengths_real    = {'bot': np.full(3, self.segment_length), 'top': np.full(3, self.segment_length)}
        self.lengths_desired = {'bot': np.full(3, self.segment_length), 'top': np.full(3, self.segment_length)}
        # integral value for each tendon
        self.integral = {'bot': np.zeros(3), 'top': np.zeros(3)}
        # timing, also needed for integrals
        init_time = self.get_clock().now().nanoseconds
        self.last_measurement_time = {'bot': init_time, 'top': init_time}

    # * parameter change callback
    def callback_param_handler(self, event: ParameterEvent) -> None:
        self.get_logger().info('Changed a parameter for this node.')

    # * callbacks for incoming lengths
    def callback_real(self, msg: Float64MultiArray, segment: str) -> None:
        self.lengths_real[segment] = np.asarray(msg.data)
        self.compute_and_publish(segment)

    def callback_desired(self, msg: Float64MultiArray, segment: str) -> None:
        self.lengths_desired[segment] = np.asarray(msg.data)
        self.compute_and_publish(segment)

    # * main control function
    def compute_and_publish(self, segment: str) -> None:
        # calculate error
        error = self.lengths_desired[segment] - self.lengths_real[segment]
        # get gains
        k_p = self.get_parameter('p_factor').value
        k_i = self.get_parameter('i_factor').value
        # update integral
        self.update_integral_of_error(error, segment)
        integral = self.integral[segment]
        # PI output is a correction on top of desired — absolute position
        correction = k_p * error + k_i * integral
        output = self.lengths_desired[segment] + correction
        # publish corrected absolute position
        msg = Float64MultiArray()
        msg.data = output.tolist()
        self.publisher_select[segment].publish(msg)
        # publish diagnostics
        self.publish_diagnostics()

    # * integral update
    def update_integral_of_error(self, error: np.ndarray, segment: str) -> None:
        windup = self.get_parameter('windup_limit').value
        dt = self.time_since_last_measurement(segment)
        dt = min(dt, 0.1)  # clamp first dt spike
        self.integral[segment] = np.clip(
            self.integral[segment] + error * dt,
            [-windup]*3, [windup]*3)

    # * timing helper
    def time_since_last_measurement(self, segment: str) -> float:
        current_time = self.get_clock().now().nanoseconds
        delta_t = (current_time - self.last_measurement_time[segment]) * 1e-9
        self.last_measurement_time[segment] = current_time
        return delta_t

    # * diagnostics
    def publish_diagnostics(self) -> None:
        error_bot = self.lengths_desired['bot'] - self.lengths_real['bot']
        error_top = self.lengths_desired['top'] - self.lengths_real['top']
        error_all = np.concatenate((error_bot, error_top))
        msg_abs = Float64()
        msg_abs.data = float(np.linalg.norm(error_all))
        self.publisher_absolute_error.publish(msg_abs)
        msg_avg = Float64()
        msg_avg.data = float(np.mean(np.abs(error_all)))
        self.publisher_average_error.publish(msg_avg)

def main():
    rclpy.init()
    mynode = Controller()
    rclpy.spin(mynode)
    mynode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()