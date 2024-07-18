"""This is a launch file that calls the two launch files and the example script to make the simulated robot
move in CoppeliaSim. For more details, refer to the repository's README.md"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    dummy_robot_example_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sas_robot_driver_ur'), 'launch'),
            '/dummy_robot_example_launch.py'])
    )
    composed_with_coppeliasim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sas_robot_driver_ur'), 'launch'),
            '/composed_with_coppeliasim_launch.py'])
    )
    return LaunchDescription([
        dummy_robot_example_launch,
        composed_with_coppeliasim_launch,
        Node(
            package='sas_robot_driver_ur',
            executable='joint_interface_example.py',
            output='screen',
            emulate_tty=True,
            name='sas_robot_driver_ur_joint_interface_example'
        )
    ])
