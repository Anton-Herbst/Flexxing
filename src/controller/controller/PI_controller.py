#! /usr/bin/env python3

"""
* This script combines the tendon error calculation and PI controller into one node.
* It takes real configuration (from IMU) and desired configuration (from trajectory)
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
        # subscribe to real configuration from sensor package 
        self.subscription_real_bot     = self.create_subscription(Float64MultiArray, '/pc/gen_coords_imu_acc_bot',        lambda msg: self.callback_real(msg, 'bot'),    10)
        self.subscription_real_top     = self.create_subscription(Float64MultiArray, '/pc/gen_coords_imu_acc_top',        lambda msg: self.callback_real(msg, 'top'),    10)
        # subscribe to desired configuration
        self.subscription_desired  = self.create_subscription(Float64MultiArray, '/pc/controller/trajectory', self.callback_desired, 10)
        # publishers that will fire off configuration based on current error
        self.publisher_controller = self.create_publisher(Float64MultiArray, '/pc/controller/output', 10)
        # publish error diagnostics
        self.publisher_absolute_error  = self.create_publisher(Float64, '/pc/controller/absolute_error', 10)
        self.publisher_average_error   = self.create_publisher(Float64, '/pc/controller/average_error',  10)

        # * Geometric Parameters
        self.segment_length = self.declare_parameter('L_segment',    0.12,   ParameterDescriptor(description='Neutral segment length.')).value
        self.d = self.declare_parameter('d', 0.018).value

        # * Parameters for controller
        self.declare_parameter('p_factor',     0.4,  ParameterDescriptor(description='P Value for the controller.'))
        self.declare_parameter('i_factor',     0.01,    ParameterDescriptor(description='I Value for the controller.'))
        self.declare_parameter('windup_limit', 0.1, ParameterDescriptor(description='Limit for the I gain.'))
        self.param_handler = ParameterEventHandler(self)
        self.param_event_callback_handle = self.param_handler.add_parameter_event_callback(self.callback_param_handler)

        # * Variables
        # placeholder configuration - start at neutral
        self.configuration_real    = {'bot': np.zeros(2), 'top': np.zeros(2)}
        self.configuration_desired = {'bot': np.zeros(2), 'top': np.zeros(2)}
        # integral value for each tendon
        self.integral = {'bot': np.zeros(2), 'top': np.zeros(2)}
        # timing, also needed for integrals - timer drives dt now, so track last time
        self.last_time = self.get_clock().now().nanoseconds
        # defines how many last values are applied to the low pass filter
        keepinmind = 3
        self.buffer_error = {segment: {i: np.zeros(keepinmind) for i in range(2)} for segment in ('bot', 'top')}
        self.buffer_average_error = np.zeros(keepinmind)
        self.buffer_absolute_error = np.zeros(keepinmind)

        # * Timer - decouples control from async subscription callbacks
        # callbacks only store the latest data; the timer runs the actual PI loop at a fixed rate
        hz = 100
        self.timer = self.create_timer(1.0 / hz, self.timer_callback)

    # * parameter change callback
    def callback_param_handler(self, event: ParameterEvent) -> None:
        self.get_logger().info('Changed a parameter for this node.')

    # * callback for measured configuration
    def callback_real(self, msg: Float64MultiArray, segment: str) -> None:
        delta_x, delta_y, _ = msg.data
        self.configuration_real[segment] = np.asarray([delta_x, delta_y])

    # * callback for desired configuration
    def callback_desired(self, msg: Float64MultiArray) -> None:
        delta_x_bot, delta_y_bot, delta_x_top, delta_y_top = msg.data
        self.configuration_desired['bot'] = np.asarray([delta_x_bot, delta_y_bot])
        self.configuration_desired['top'] = np.asarray([delta_x_top, delta_y_top])

    # * timer callback - runs PI loop at a fixed rate
    def timer_callback(self) -> None:
        # get time since last callback, used for integral later on
        dt = self.time_since_last()
        # prepare a list that can be published
        control_output = [0.0]*4
        # populate the list with the current control outputs
        control_output[0], control_output[1] = self.compute_control_output('bot', dt)
        control_output[2], control_output[3] = self.compute_control_output('top', dt)
        # publish controller output to later be converted by G2L_controller
        msg = Float64MultiArray()
        msg.data = control_output
        self.publisher_controller.publish(msg)
        # publish diagnostics once per callback, covering both segments
        self.publish_diagnostics()

    # * main control function
    def compute_control_output(self, segment: str, dt: float) -> tuple[float, float]:
        # calculate error
        reading = self.configuration_desired[segment] - self.configuration_real[segment]
        # get rid of any leftover noise
        error = self.low_pass_filter_error(reading, segment)
        # get gains
        k_p = self.get_parameter('p_factor').value
        k_i = self.get_parameter('i_factor').value
        # update integral
        self.update_integral_of_error(error, segment, dt)
        integral = self.integral[segment]
        # PI output is a correction on top of the desired correction
        correction = k_p * error + k_i * integral
        output = self.configuration_desired[segment] + correction
        return output[0], output[1]

    # * integral update
    def update_integral_of_error(self, error: np.ndarray, segment: str, dt: float) -> None:
        windup = self.get_parameter('windup_limit').value
        self.integral[segment] = np.clip(
            self.integral[segment] + error * dt,
            [-windup]*2, [windup]*2)

    # * timing helper - measures elapsed time since last timer callback
    def time_since_last(self) -> float:
        current_time = self.get_clock().now().nanoseconds
        delta_t = (current_time - self.last_time) * 1e-9
        # clamp to guard against huge spikes at the start or any hangups
        delta_t = min(delta_t, 0.1)
        # set current time as last time it was checked
        self.last_time = current_time
        return delta_t

    # * errors further helping with visualizing
    def publish_diagnostics(self) -> None:
        error_bot = self.configuration_desired['bot'] - self.configuration_real['bot']
        error_top = self.configuration_desired['top'] - self.configuration_real['top']
        error_all = np.concatenate((error_bot, error_top))
        self.publish_errors(error_all)

    # * filter so controller is more reliable
    def low_pass_filter_error(self, input:np.ndarray, segment:str) -> np.ndarray:
        # prepare a list to be returned
        filtered = np.zeros(2)
        for i in range(2):
            # take the relevant buffer and shift all values to the left
            self.buffer_error[segment][i] = np.roll(self.buffer_error[segment][i], -1)
            # place the newest one to the furthest right
            self.buffer_error[segment][i][-1] = input[i]
            # get the mean as the filters output
            filtered[i] = np.mean(self.buffer_error[segment][i])
        return filtered
    
    def publish_errors(self, error_all: np.ndarray) -> None:
        msg_abs = Float64()
        absolute_error = float(np.linalg.norm(error_all))
        self.buffer_absolute_error = np.roll(self.buffer_absolute_error, -1)
        self.buffer_absolute_error[-1] = absolute_error
        msg_abs.data = np.mean(self.buffer_absolute_error)
        self.publisher_absolute_error.publish(msg_abs)
        msg_avg = Float64()
        average_error = float(np.mean(np.abs(error_all)))
        self.buffer_average_error = np.roll(self.buffer_average_error, -1)
        self.buffer_average_error[-1] = average_error
        msg_avg.data = np.mean(self.buffer_average_error)
        self.publisher_average_error.publish(msg_avg)

def main():
    rclpy.init()
    mynode = Controller()
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