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
            name='visualize_imu_acc_tip',
            output='screen'
        ),
    ])