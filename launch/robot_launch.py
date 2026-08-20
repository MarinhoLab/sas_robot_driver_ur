import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name = LaunchConfiguration('name')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='ur3e'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(get_package_share_directory('sas_robot_driver_ur'), 'config', 'config.yaml')
        ),
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_robot_driver_ur',
            executable='sas_robot_driver_ur_node',
            name=name,
            parameters=[config_file]
        ),
    ])
