from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='sensor',
            executable='publish_IMU_acc_tip',       # this node subs raw sensor data and pubs it transformed
            name='publish_IMU_acc_tip',
            output='screen'
        ),
        Node(
            package='configuration',
            executable='bending_IMU_acc_tip',     # this node takes the transformed data and calculates the bending 
            name='bending_IMU_acc_tip',
            output='screen'
        ),
        Node(
            package='visualize',
            executable='visualize_IMU_acc_tip',     # this node takes the bending to visualize the direction of the tip
            name='visualize_IMU_acc_tip',
            output='screen'
        ),
    ])