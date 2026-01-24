# robot_controller

This package configures and launches the **ros2_control** infrastructure for the robot. It does not contain application logic, but provides:

* Controller configuration (YAML)
* A launch file to start `ros2_control`, controllers, and state publishing

The package acts as the bridge between the robot description (URDF) and higher-level motion components such as MoveIt or custom controllers.

---

## Package Structure

```
robot_controller/
├── include/
├── src/
├── config/
│   └── robot_controllers.yaml
├── launch/
│   └── robot_controllers.launch.py
└── package.xml
└── CMakeList.txt
```

---

## Purpose of the Package

**Intent:**

* Start the ROS 2 control framework for the robot
* Load and configure controllers for arm joints and the gripper
* Publish joint states for visualization and downstream consumers

**Design assumptions:**

* The robot URDF is provided by the `robot_description` package
* Controllers are position-controlled
* This package is used both in simulation and on real hardware

---

## Launch File Overview

The launch file performs the following steps:

1. Generates the `robot_description` parameter by processing a XACRO file
2. Starts `robot_state_publisher`
3. Starts the `ros2_control_node` (controller manager)
4. Spawns the following controllers:

   * `joint_state_broadcaster`
   * `arm_controller`
   * `gripper_controller`

This ensures that controller startup order is correct and all required interfaces are available.

---

## Controller Configuration (`robot_controllers.yaml`)

### Controller Manager

```yaml
controller_manager:
  ros__parameters:
    update_rate: 10
```

* Sets the control loop update rate (Hz)
* Applies globally to all controllers

---

### Arm Controller

```yaml
arm_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
```

* Uses a `JointTrajectoryController`
* Controls three arm joints
* Operates in **position control mode**

```yaml
    open_loop_control: true
```

* Assumes the hardware executes commands without feedback correction
* Typical for simple servo or simulation setups

---

### Gripper Controller

```yaml
gripper_controller:
  ros__parameters:
    joint: gripper
```

* Uses a `GripperActionController`
* Accepts position commands via ROS action interface
* Intended for simple open/close behavior

---

### Joint State Broadcaster

```yaml
joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster
```

* Publishes `/joint_states`
* Required for RViz, robot_state_publisher, and MoveIt

---

## Typical Usage

```bash
ros2 launch robot_controller robot_controllers.launch.py
```

This launch file is usually started:

* before MoveIt
* before hardware interface nodes
* as part of a larger system launch

---

## Summary

* `robot_controller` configures **ros2_control** for the robot
* Contains no motion logic, only configuration and startup
* Defines controllers for:

  * arm joints
  * gripper
  * joint state publishing
* Acts as a core infrastructure package in the robot software stack
