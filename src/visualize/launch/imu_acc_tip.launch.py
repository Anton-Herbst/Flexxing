from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='sensor',
            executable='publish_imu_acc_tip',       # this node subs raw imu linear acceleration vector and pubs it transformed
            name='publish_imu_acc_tip',             # to the local robots coordinate system
            output='screen'
        ),
        Node(
            package='sensor',
            executable='generalized_coords_imu_acc_tip',       # this node takes the transformed vector and calculates the bending 
            name='generalized_coords_imu_acc_tip',             # for the segment (here only one)
            output='screen'
        ),
        Node(
            package='visualize',
            executable='visualize_imu_acc_tip',     # this node takes the bending to visualize the direction of the tip
            name='visualize_imu_acc_tip',           # ! here forward kinematics is used should be exported to package kinematics/G2X_imu_acc_tip.py
            output='screen'
        ),
        Node(
            package='servos',
            executable='static',
            name='servo_setting',            # this node will make controlling the robot easier combined with rqt_gui
            output='screen'
        ),
        ExecuteProcess(
            cmd=['rqt'],                    # this opens rqt_gui so that the robot can be used as RC toy
            output='screen'
        ),
    ])