from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument( # this defines the trajectory which will be followed
            'traj',
            default_value='circle',
            description='Trajectory name'
        ),
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
        Node(   # this node will publish generalized coordinates as a target
            package='controller',
            executable='trajectory_gen',
            name='trajectory_gen',
            parameters=[{ 'trajectory_name':  LaunchConfiguration('traj') }],
            output='screen',
        ),
        Node(   # this node will give an output based on the error between target and real configuration
            package='controller',
            executable='PI_controller',
            name='PI_controller',
            output='screen'
        ),
        Node(
            package='kinematics',
            executable='G2L_controller',
            name='Tendon_Calculator',
            output = 'screen',
        ),
        Node(  # this node will act based on the commanded tendon lengths
            package='servos',
            executable='plant',
            name='plant',
            parameters = [{'control_mode': 'closed_loop'}],
            output='screen'
        ),
        Node(   # this node will show where the trajectory is in space
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
        Node(   # this will publish how far off the tip is in the cartesian system
            package='visualize',
            executable='cartesian_error',
            name = 'cartesian_error',
            output='screen'
        ),
    ])