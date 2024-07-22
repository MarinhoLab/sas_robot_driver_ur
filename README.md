# sas_robot_driver_ur


## Running the joint space example in CoppeliaSim

*The CoppeliaSim scene must be fixed as the UR3 in CoppeliaSim and the UR3e we have seem to have different joint directions.*

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