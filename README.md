# sas_robot_driver_ur

## Initial setup

This is for a system that has all the [prerequisites](https://github.com/SmartArmStack/smart_arm_stack_ROS2/tree/jazzy) installed. 

### Cloning this repository with all SmartArmStack_ROS2 dependencies

```commandLine
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
git clone https://github.com/SmartArmStack/smart_arm_stack_ROS2.git --recursive
```

### Building and sourcing the repository

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Interfacing your code with this package

This package is intended to expose joint positions of the robot to ROS2. It is based on the [sas_robot_driver](https://github.com/SmartArmStack/sas_robot_driver/tree/jazzy) server--client topology. 

### Server

Each robot node will create a server with ROS2 topics. That is completely managed by [sas_robot_driver](https://github.com/SmartArmStack/sas_robot_driver/tree/jazzy). 
You **must** use a launch file to create your own servers, if needed. For guidance, use `launch/real_robot_example_launch.py` and modify as needed.

For most users, you will only need to modify the ip address to match the ip address of your robot.

https://github.com/MarinhoLab/sas_robot_driver_ur/blob/528d18bcf50ef257df8e9c05ef1df9dc15b26eb4/launch/real_robot_example_launch.py#L28

If you have multiple robots, remember to change each one to have a unique name.

https://github.com/MarinhoLab/sas_robot_driver_ur/blob/528d18bcf50ef257df8e9c05ef1df9dc15b26eb4/launch/real_robot_example_launch.py#L26

### Client

After your server is running, you can obtain current joint positions and send joint position commands.
All those are managed through ROS2 topics. The benefit is that the [sas_robot_driver](https://github.com/SmartArmStack/sas_robot_driver/tree/jazzy) library makes that transparent to the user, so you will not have to create your own subscribers/publishers manually.

An example on how to do that using `rclpy` is available at 

```
scripts/joint_interface_example.py
```

#### Getting joint positions

https://github.com/MarinhoLab/sas_robot_driver_ur/blob/528d18bcf50ef257df8e9c05ef1df9dc15b26eb4/scripts/joint_interface_example.py#L62

#### Sending joint position commands

https://github.com/MarinhoLab/sas_robot_driver_ur/blob/528d18bcf50ef257df8e9c05ef1df9dc15b26eb4/scripts/joint_interface_example.py#L72

### Ok, but what is this ROS composer thing?

When using CoppeliaSim, you will notice that we're not interfacing directly with the robot node. You can see that from the name

https://github.com/MarinhoLab/sas_robot_driver_ur/blob/528d18bcf50ef257df8e9c05ef1df9dc15b26eb4/scripts/joint_interface_example.py#L51

and the indirection in this launch file that runs a `sas_robot_driver_ros_composer_node`

https://github.com/MarinhoLab/sas_robot_driver_ur/blob/528d18bcf50ef257df8e9c05ef1df9dc15b26eb4/launch/composed_with_coppeliasim_launch.py#L18

The composer node has two main roles. First, it allows us to abstract many different devices into a single serial robot. This is important for [complex systems](https://github.com/AISciencePlatform). It also allows us to reflect robot state in CoppeliaSim, as a virtual twin.

These are specified in the following parameters in the launch file and most parameters are probably self-evident. The `vrep_dynamically_enabled` is related to a joint being passive or active in CoppeliaSim and this depends on your scene.

https://github.com/MarinhoLab/sas_robot_driver_ur/blob/9fa2f2b5a4aa6a84becec0748a70786772230a76/launch/composed_with_coppeliasim_launch.py#L25-L29

Please note that this means that CoppeliaSim can be executed in any computer accessible with the ip address and port specified, using, in a transparent manner, the legacy [remoteAPI](https://manual.coppeliarobotics.com/en/legacyRemoteApiOverview.htm). It can be a completely separate computer running Windows, for example.

## Working with CoppeliaSim

https://github.com/user-attachments/assets/bfee1148-bfe3-4425-80da-04fcd65d2b18



## Working with the real robot

For using the real robot, you **must** have the risk assessments in place. This guide is meant to be helpful but holds absolutely no liability whatsoever. More details are available in the software license.

This code will move the robot. Be sure that the workspace is free and safe for operation.

1. Be sure that the teaching pendant is in `Remove Control` mode.  
2. Split the terminator into four screens. Now, the order matters.

| `a` | `b` |
|-----|-----|
| `c` | `d` |

3. In `a`, run the CoppeliaSim scene `scenes/UR3_470rev4.ttt` and start the simulation.
4. In `b`, run `ros2 launch sas_robot_driver_ur real_robot_example_launch.py`
   - The emergency button must be held at all times.
   - After some seconds of initialization, the robot will be active. 
6. In `c`, run `ros2 launch sas_robot_driver_ur composed_with_coppeliasim_launch.py`. This will connect the CoppeliaSim scene with the ros2 code.
7. In `d`, run `ros2 run sas_robot_driver_ur joint_interface_example.py`. The robot will move in a sine wave in joint space, with respect to its initial joint values.


https://github.com/user-attachments/assets/5902f735-6c42-4825-a552-58e565bbf3f3

## Troubleshooting tips

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/507
