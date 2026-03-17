from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',
            executable='trajectory_gen',
            name='trajectory_gen',
            output='screen',
        ),
        Node(
            package='kinematics',
            executable='G2X_trajectory',
            name='trajectory_cartesian',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
            output='screen'
        ),
    ])