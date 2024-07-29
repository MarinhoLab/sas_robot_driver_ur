"""
Run this script in a different terminal window or tab. Be ready to close this, as this activates the real robot if the
connection is successful.
"""
import os.path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Examples from UR
    # const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
    # const std::string SCRIPT_FILE = "resources/external_control.urscript";
    # const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
    # const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";
    # const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";

    return LaunchDescription([
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_robot_driver_ur',
            executable='sas_robot_driver_ur_node',
            name='ur_1',
            parameters=[{
                "ip": "192.170.10.22",
                "script_file": os.path.join(get_package_share_directory("sas_robot_driver_ur"),
                                            "external_control.urscript"),
                "output_recipe": os.path.join(get_package_share_directory("sas_robot_driver_ur"),
                                              "rtde_output_recipe.txt"),
                "input_recipe": os.path.join(get_package_share_directory("sas_robot_driver_ur"),
                                             "rtde_input_recipe.txt"),
                "calibration_checksum": "calib_12788084448423163542",
                "joint_limits_min": [-360.0, -360.0, -360.0, -360.0, -360.0, -720.0],  # The last joint has no limit
                "joint_limits_max": [360.0, 360.0, 360.0, 360.0, 360.0, 720.0],  # The last joint has no limit
                "thread_sampling_time_sec": 0.001
            }]
        ),

    ])
