import os.path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Examples from UR
    # const std::string DEFAULT_ROBOT_IP = "192.168.56.101";
    # const std::string SCRIPT_FILE = "resources/external_control.urscript";
    # const std::string OUTPUT_RECIPE = "examples/resources/rtde_output_recipe.txt";
    # const std::string INPUT_RECIPE = "examples/resources/rtde_input_recipe.txt";
    # const std::string CALIBRATION_CHECKSUM = "calib_12788084448423163542";
    name = LaunchConfiguration('name')
    ip = LaunchConfiguration('ip')
    turn_robot_off_on_connect = LaunchConfiguration('turn_robot_off_on_connect')
    turn_robot_off_on_disconnect = LaunchConfiguration('turn_robot_off_on_disconnect')
    calibration_checksum = LaunchConfiguration('calibration_checksum')

    return LaunchDescription([
        DeclareLaunchArgument(
            'name',
            default_value='kuka_1'
        ),
        DeclareLaunchArgument(
            'ip',
            default_value='192.170.10.22'
        ),
        DeclareLaunchArgument(
            'turn_robot_off_on_connect',
            default_value='True'
        ),
        DeclareLaunchArgument(
            'turn_robot_off_on_disconnect',
            default_value='True'
        ),
        DeclareLaunchArgument(
            'calibration_checksum',
            default_value='calib_12788084448423163542'
        ),
        Node(
            output='screen',
            emulate_tty=True,
            package='sas_robot_driver_ur',
            executable='sas_robot_driver_ur_node',
            name=name,
            parameters=[{
                "ip": ip,
                "script_file": os.path.join(get_package_share_directory("sas_robot_driver_ur"),
                                            "external_control.urscript"),
                "output_recipe": os.path.join(get_package_share_directory("sas_robot_driver_ur"),
                                              "rtde_output_recipe.txt"),
                "input_recipe": os.path.join(get_package_share_directory("sas_robot_driver_ur"),
                                             "rtde_input_recipe.txt"),
                "calibration_checksum": calibration_checksum,
                "joint_limits_min": [-360.0, -360.0, -360.0, -360.0, -360.0, -720.0],  # The last joint has no limit
                "joint_limits_max": [360.0, 360.0, 360.0, 360.0, 360.0, 720.0],  # The last joint has no limit
                "thread_sampling_time_sec": 0.002, # Robot thread is at 500 Hz,
                "turn_robot_off_on_connect": turn_robot_off_on_connect,
                "turn_robot_off_on_disconnect": turn_robot_off_on_disconnect
            }]
        ),

    ])
