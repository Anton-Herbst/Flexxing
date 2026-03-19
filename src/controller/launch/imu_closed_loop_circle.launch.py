from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(   # this node transforms sensor values to be analyzed
            package='sensor',
            executable='publish_imu_acc_all',
            name='publish_imu_acc_all',
            output='screen',
        ),
        Node(   # this node gives out generalized coordinates
            package='sensor',
            executable='generalized_coords_imu_acc_all',
            name='generalized_coords_imu_acc_all',
            output='screen',            
        ),
        Node(   # this node will publish the real tendon lengths from sensed coordinates
            package='kinematics',
            executable = 'G2L_imu_acc_all',
            name='real_tendon_lengths',
            output='screen',
        ),
        Node(   # this node will publishe generalized coordinates as a target
            package='controller',
            executable='trajectory_gen',
            parameters=[{ 'trajectory_name': 'circle_xy' }],
            name='trajectory_gen',
            output='screen',
        ),
        Node(   # this node will publish target tendon lengths from the trajectory
            package='kinematics',
            executable = 'G2L_trajectory',
            name='trajectory_tendon_lengths',
            output='screen',
        ),
        Node(   # this node will publish the difference between target and real tendon lenghts
            package='controller',
            executable='publish_tendon_error',
            name='publish_tendon_error',
            output='screen'
        ),
        Node(   # this node will give an output based on the error
            package='controller',
            executable='PI_controller',
            name='PI_controller',
            output='screen'
        ),
        Node(   # this node will act based on the controller output -> motor control
            package='servos',
            executable='plant',
            name='plant',
            output='screen'
        ),
    ])