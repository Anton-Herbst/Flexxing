from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('calibration'),
        'config',
        'pose_cal.yaml'
    )

    return LaunchDescription([
        Node(
            package='calibration',
            executable='mag_tip',
            name='mag_tip',
            parameters=[config],            # this launch file was needed so the parameters in config can be accessed
            output='screen'
        )
    ])