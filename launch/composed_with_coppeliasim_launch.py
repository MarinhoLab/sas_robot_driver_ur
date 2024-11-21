"""This launch file depends on the robot driver mentioned, `kuka_1`, be active. In addition, the correct scene must
be loaded in CoppeliaSim and the simulation must be started. If there are connection issues, restarting the simulation
(not the entire program, just stopping and starting the simulation) might do the trick."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joint_names = [
        "/UR3/joint",
        "/UR3/link/joint",
        "/UR3/link/joint/link/joint",
        "/UR3/link/joint/link/joint",
        "/UR3/link/joint/link/joint/link/joint",
        "/UR3/link/joint/link/joint/link/joint/link/joint",
    ]

    return LaunchDescription([
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_composer_node',
            output='screen',
            emulate_tty=True,
            name='sas_robot_driver_ur_composed_with_coppeliasim',
            parameters=[{
                "robot_driver_client_names": ["ur_1"],
                "use_real_robot": True,
                "use_coppeliasim": True,
                "vrep_robot_joint_names": joint_names,
                "vrep_ip": "127.0.0.1",
                "vrep_port": 23000,
                "vrep_dynamically_enabled": True,
                "override_joint_limits_with_robot_parameter_file": False,
                "thread_sampling_time_sec": 0.001
            }]
        )
    ])
