# robot_description Package

## Overview

The `robot_description` package contains the complete **kinematic, visual, and control description** of the robot. It is the single source of truth for:

* Robot geometry (meshes)
* Kinematic structure (URDF)
* ros2_control integration
* RViz visualization setup

This package is intended to be reused by visualization, control, and planning components like MoveIt later on.

---

## Package Structure

```
robot_description/
├── launch/
│   └── display.launch.py
│   └── gazebo.launch.py (Comming soon.)
├── meshes/
│   └── *.stl
├── rviz/
│   └── display.rviz
└── urdf/
    ├── robot.urdf.xacro
    └── robot_ros2_control.xacro
    └── robot_gazebo.xacro (Comming soon.)
```

---

## URDF / XACRO Files

### robot.urdf.xacro

This file defines the **physical and kinematic structure** of the robot:

* All robot links and joints
* Joint limits, axes, and origins
* Visual meshes and inertial properties
* End-effector (`ee_link`) definition
* Mimic joint for the gripper

The robot consists of:

* A fixed `world` frame
* A 3-DOF revolute arm (`joint_1`, `joint_2`, `joint_3`)
* A two-finger gripper using a mimic joint (`gripper`, `gripper2`)

All joint names are consistent with:

* `/joint_states`
* ros2_control
* GUI and hardware interfaces

### robot_ros2_control.xacro

This file contains the **ros2_control configuration** and is included by `robot.urdf.xacro`.

Key aspects:

* Uses `mock_components/GenericSystem` as hardware interface
* Designed for simulation and visualization without real hardware
* Defines command and state interfaces for each joint
* Joint limits are aligned with URDF limits
* Gripper mimic behavior is mirrored in ros2_control

> Gazebo or real hardware plugins are **not yet included** and are planned as future work.

---

## Meshes

The `meshes/` directory contains all STL files referenced by the URDF:

* Base
* Arm segments
* Gripper components

Meshes are scaled uniformly (`0.01`) to match the URDF coordinate system.

---

## RViz Configuration

### rviz/display.rviz

This file provides a preconfigured RViz setup including:

* RobotModel display
* TF visualization
* Fixed frame set to `world`

It allows immediate visualization of the robot when launched.

---

## Launch File

### display.launch.py

This launch file starts the complete **visualization stack**:

* Loads the robot description via XACRO
* Publishes TF using `robot_state_publisher`
* Starts the GUI node
* Launches RViz with a predefined configuration

It is intended for:

* Development
* Debugging
* Demonstrations

---

## Usage

```bash
ros2 launch robot_description display.launch.py
```

Optional custom URDF:

```bash
ros2 launch robot_description display.launch.py model:=/absolute/path/to/robot.urdf.xacro
```

---

## Future Work

The following components are intentionally not part of the current package:

* Gazebo simulation integration
* Real hardware ros2_control plugins for Gazebo
* Collision geometry optimization

These features are planned to be added in future development stages to be able to not just visualize but also simulate the robot.
