from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument( # this defines the trajectory which will be followed
            'traj',
            default_value='circle_xy',
            description='Trajectory name'
        ),
        Node(   # this node transforms sensor values to be analyzed
            package='sensor',
            executable='publish_imu_acc_all',
            name='publish_imu_acc_all',
            output='screen',
        ),
        Node(   # this node gives out generalized coordinates from sensors
            package='sensor',
            executable='generalized_coords_imu_acc_all',
            name='generalized_coords_imu_acc_all',
            output='screen',            
        ),
        Node(   # this node will make a target trajectory
            package='controller',
            executable='trajectory_gen',
            name='trajectory_gen',
            parameters=[{ 'trajectory_name':  LaunchConfiguration('traj') }],
            output='screen',
        ),
        Node(   # this node will publish target tendon lengths from the trajectory
            package='kinematics',
            executable = 'G2L_trajectory',
            name='trajectory_tendon_lengths',
            output='screen',
        ),
        Node(   # this node will act based on target lengths
            package='servos',
            executable='plant',
            parameters = [{'control_mode': 'open_loop'}],
            name='plant',
            output='screen'
        ),
        Node(   # this node is only called to track the error
            package='controller',
            executable='PI_controller',
            name='PI_controller',
            output='screen'
        ),
        Node(   # this node will show where the trajectory is
            package = 'kinematics',
            executable = 'G2X_trajectory',
            name = 'trajectory_position',
            output = 'screen',
        ),
        Node(   # this node will show where the endeffector really is according to the imu sensor
            package='kinematics',
            executable = 'G2X_imu_acc_all',
            name = 'endeffector_position',
            output = 'screen',       
        ),
    ])