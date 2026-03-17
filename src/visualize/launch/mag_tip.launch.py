from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='sensor',
            executable='publish_mag_tip',       # this node subs raw imu linear acceleration vector and pubs it transformed
            name='publish_mag_tip',             # to the local robots coordinate system
            output='screen'
        ),
        Node(
            package='sensor',
            executable='generalized_coords_mag_tip',       # this node takes the transformed vector and calculates the bending 
            name='generalized_coords_mag_tip',             # for the segment (here only one)
            output='screen'
        ),
        Node(
            package='kinematics',
            executable='G2X_mag_tip',       # this node takes the bending and calculates the position of the tip
            name='G2X_mag_tip',             # using forward kinematics (only one segment since only one sensor was used)
            output='screen'
        ),
        Node(
            package='visualize',
            executable='mag_tip',     # this node visualizes the endeffector vector
            name='mag_tip',           
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