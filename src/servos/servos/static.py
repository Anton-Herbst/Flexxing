#!/usr/bin/env python3

"""
* This file is for manually setting servo values.
--------------------------------------------------------------------------------------------------------
There is already a topic which can be directly published to via

ros2 topic pub /teensy_hub/servo_pos soro_msgs/msg/ServoCommands "{servo_micros: [1500, 1500, 1500, 1500, 1500, 1500, 1500, 15000, 1500, 1500, 1500, 1500]}"

This is kind of annoying to type each time so this code will make that easier by using parameters.
After running this with "ros2 run servos static" 
use "ros2 run rqt_gui rqt_gui" with the parameter plugin.
There you can easily slide each servos value to see what happens
"""

import rclpy                                                    # to be able to use ROS with python
from rclpy.node import Node                                     # ROS node creation
from soro_msgs.msg import ServoCommands                         # datatype for publishing servos
import rclpy.parameter                                          # enables the use of ROS Parameters
from rcl_interfaces.msg import ParameterDescriptor              # for describing parameter
from rcl_interfaces.msg import IntegerRange                     # limits servo range   
from rcl_interfaces.msg import SetParametersResult              # allow to set new param values

class Static(Node):
    # * function called on creation
    def __init__(self):
        # "it insists upon itself"
        super().__init__('servo_setting')
        # * Parameters
        # how often the servo commands should be send
        self.hz = 60
        # PWM values for the 6 servos
        self.order = ['top1','bot1','top2','bot2','top3','bot3']
        # acceptable values are these:
        limits = IntegerRange(from_value = 1000, step = 1, to_value = 2000)
        # create 6 parameters, one for each servo
        self.declare_parameters(namespace='', parameters=[
            ('bot1', 1500, ParameterDescriptor(description='Initial PWM value for bottom motor 1', integer_range=[limits])),
            ('bot2', 1500, ParameterDescriptor(description='Initial PWM value for bottom motor 2', integer_range=[limits])),
            ('bot3', 1500, ParameterDescriptor(description='Initial PWM value for bottom motor 3', integer_range=[limits])),
            ('top1', 1500, ParameterDescriptor(description='Initial PWM value for top motor 1', integer_range=[limits])),
            ('top2', 1500, ParameterDescriptor(description='Initial PWM value for top motor 2', integer_range=[limits])),
            ('top3', 1500, ParameterDescriptor(description='Initial PWM value for top motor 3', integer_range=[limits])),
        ])
        # make a struct so that values[name] will give the appropriate PWM 
        self.values = {name: self.get_parameter(name).value for name in self.order}
        # handles changes to parameter values
        self.add_on_set_parameters_callback(self.param_handler_callback)
        # * ROS topics
        # link into the servo topic to command the robots actuators
        self.servo_pub = self.create_publisher(
            msg_type    = ServoCommands,
            topic       = '/teensy_hub/servo_pos',
            qos_profile = 10)
        self.timer = self.create_timer(
            timer_period_sec = 1/self.hz,
            callback         = self.timer_callback,)
    # * Callback function on parameter change
    def param_handler_callback(self, param: rclpy.parameter.Parameter):
        # go through the changed parameters to change the value struct
        for p in param:
            self.values[p.name] = p.value
        # print out changed servo commands
        print(f"\n New Servo Commands:")
        for name in self.order:
            print(self.values[name])
        return SetParametersResult()
    # * Callback function on timer
    def timer_callback(self):
        # create a variable to pass, replace the relevant servo inputs 
        commands = [1500] * 12
        commands[0] = self.values['top1']
        commands[1] = self.values['bot1']
        commands[2] = self.values['top2']
        commands[3] = self.values['bot2']
        commands[4] = self.values['top3']
        commands[5] = self.values['bot3']
        # create a msg type for the servos
        servo_msg = ServoCommands()
        # fill the msg with the created custom values
        servo_msg.servo_micros = commands
        # publish the full msg to the targeted servo
        self.servo_pub.publish(servo_msg)
    
def main():
    rclpy.init()
    mynode = Static()
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