"""This launches a dummy robot node that reflects on the outputs any inputs that it was given. It is used to simulate
a robot and keep the state of it for simulations using CoppeliaSim and dry runs."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sas_robot_driver',
            executable='sas_robot_driver_ros_example',
            name='ur_1',
            parameters=[{
                "robot_name": "ur_1",
                "joint_limits_min": [-360.0, -360.0, -360.0, -360.0, -360.0, -720.0],  # The last joint has no limit
                "joint_limits_max": [360.0, 360.0, 360.0, 360.0, 360.0, 720.0],  # The last joint has no limit
                "initial_joint_positions": [0., 0., 0., 0., 0., 0.],
                "thread_sampling_time_sec": 0.001
            }]
        ),

    ])
