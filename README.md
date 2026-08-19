# sas_robot_driver_ur

> [!TIP]
> More information about the SmartArmStack is available in https://smartarmstack.github.io/.

> [!IMPORTANT]
> Do not clone this repository directly. See https://github.com/MarinhoLab/sas_ur_control_template

## ROS 2 Nodes & Parameters

This repository defines a single ROS 2 node. The format below scales to multiple nodes — one section per node, each with its own parameter table.

---

### Node: `sas_robot_driver_ur_node`

| Property | Value |
|---|---|
| **Executable** | `sas_robot_driver_ur_node` |
| **ROS node name** | `sas_robot_driver_ur` |
| **Description** | Main driver node for the UR robot. Reads all parameters, instantiates `RobotDriverUR` (FRI/RTDE interface to the robot), `ForceSensorServer` (publishes TCP force/torque), and `RobotDriverROS` (runs the control loop). |

#### Parameters

| Parameter | Type | Mandatory / Optional | Default | Purpose |
|---|---|---|---|---|
| `ip` | string | **Mandatory** | none — must be provided | IP address of the UR robot controller (FRI/RTDE) |
| `script_file` | string | **Mandatory** | none — must be provided | Path to the URScript external-control program uploaded to the robot (e.g. `external_control.urscript`) |
| `output_recipe` | string | **Mandatory** | none — must be provided | RTDE output recipe (e.g. `rtde_output_recipe.txt`) |
| `input_recipe` | string | **Mandatory** | none — must be provided | RTDE input recipe (e.g. `rtde_input_recipe.txt`) |
| `calibration_checksum` | string | **Mandatory** | none — must be provided | Calibration checksum for the RTDE handshake; must match the robot's (e.g. `calib_12788084448423163542`) |
| `joint_limits_min` | array of 6 doubles (degrees) | **Mandatory** | none — must be provided | Minimum joint limits; converted from deg → rad internally |
| `joint_limits_max` | array of 6 doubles (degrees) | **Mandatory** | none — must be provided | Maximum joint limits; converted from deg → rad internally |
| `thread_sampling_time_sec` | double | **Mandatory** | none — must be provided | Sampling period of the robot control-loop thread (e.g. `0.002` s = 500 Hz) |
| `turn_robot_off_on_connect` | bool | Optional | `true` | Whether to power the robot off when connecting |
| `turn_robot_off_on_disconnect` | bool | Optional | `true` | Whether to power the robot off on disconnect |

**How mandatory/optional is determined in code:**
- **Mandatory** params are read with `sas::get_ros_parameter(...)` — if missing, the node throws and fails to start.
- **Optional** params are read with `sas::get_ros_optional_parameter(..., <default>)` — they carry in-code defaults.

> **Caveat:** `RobotDriverROS` (from `sas_robot_driver`) and `ForceSensorServer` (from `sas_force_sensor`) are external packages not in this repository. Any parameters *they* declare on this same node can't be verified from this codebase — each table above covers only what that node's own code reads.
