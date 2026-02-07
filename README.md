# PI-Bot: Distributed 3-DOF Robotic Arm

## Table of Contents
1. [Project Overview & Demo](#1-project-overview--demo)
2. [System Architecture](#2-system-architecture)
3. [Key Features](#3-key-features)
4. [Hardware & Wiring](#4-hardware--wiring)
5. [Project Structure](#5-project-structure)
6. [Kinematics & MoveIt2 Configuration](#6-kinematics--moveit2-configuration)
7. [Requirements & Installation](#7-requirements--installation)
8. [Getting Started](#8-getting-started)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Project Overview & Demo <a name="1-project-overview--demo"></a>

### Demo
![PI-Bot Stacking Sequence](HIER_LINK_ZU_DEINEM_GIF_ODER_VIDEO_EINSETZEN)
*The PI-Bot autonomously performing a pick-and-place task, synchronized with its Digital Twin in RViz.*

### About the Project
The **PI-Bot** is a custom-engineered, 3D printed, 3-DOF robotic arm designed as a full-stack robotics project. It demonstrates the seamless integration of distributed software systems, real-time hardware interaction, and motion planning. 

The project covers:
*   **Mechanical Design in CAD:** 3-axis manipulator with gripper.
*   **Embedded Control:** Custom ROS2 nodes on a Raspberry Pi 4 for PWM actuation and ADC sensor readout.
*   **Advanced Middleware:** MoveIt2 for path planning and a synchronized Digital Twin.
*   **UI:** A professional PySide6/Qt GUI for orchestration.

---


## 2. System Architecture <a name="2-system-architecture"></a>

The PI-Bot is built on a **distributed ROS2 architecture**, separating high-level logic from hardware-near execution to ensure modularity and system stability.

![System Overview](./docs/images/system-overview-color.png)
*High-level overview of the distributed control loop and node communication.*

### Main PC (High-Level Logic)
The workstation acts as the "Brain" of the system, running computationally intensive nodes:
*   **GUI-Node (PySide6):** The central hub for user interaction. It subscribes to `/pot_values` for manual input and `/fk_position` for real-time monitoring. It orchestrates movements by calling the MoveIt2 Action Client.
*   **MoveIt2 Action Client:** Handles trajectory planning and inverse kinematics (IK). Once a path is planned, it publishes the trajectory points to the `/joint_states` topic.
*   **Forward Kinematics Node:** A dedicated node that processes joint angles to calculate the end-effector's XYZ position, providing feedback to the GUI.
*   **RViz Digital Twin:** Subscribes to `/joint_states` to provide a real-time 3D visualization of the robot's physical state.

### Raspberry Pi 4 (Hardware Abstraction Layer)
The Pi 4 serves as the bridge between the digital ROS2 environment and the physical hardware:
*   **Potentiometer Node:** Interfaces with the **ADS7830 ADC** via **I2C**. It reads analog signals from the three potentiometers, processes them, and publishes the values to the `/pot_values` topic.
*   **Servo Controller Node:** Subscribes to the `/joint_states` topic coming from the PC. It translates these states into commands for the **PCA9685 Servo Driver** via **I2C**, which then drives the 4x Servo Motors using PWM signals.

### Communication Layer
*   **ROS2 Middleware:** All communication between the PC and the Raspberry Pi is handled over the local network using a shared **ROS_DOMAIN_ID**.


## 3. Key Features <a name="3-key-features"></a>

*   **Hybrid Control Modes:** Switch between GUI-Slider control and **Hardware-in-the-Loop (HIL)** control using analog potentiometers.
*   **Teach-In Sequence Recorder:** Store specific poses (Cartesian or Joint-space) in a table and execute them as a complex motion sequence.
*   **Real-time State Sync:** A specialized `sync_sliders_to_movement` logic ensures the GUI and Digital Twin always reflect the physical hardware state to prevent sudden jumps.
*   **Asynchronous Processing:** Multi-threaded Python/Qt implementation to keep the UI responsive during long-running trajectory executions.

---

## 4. Hardware & Wiring <a name="4-hardware--wiring"></a>

| Component | Interface | Description |
| :--- | :--- | :--- |
| **Raspberry Pi 4** | Ethernet/WiFi | Main Brain (Distributed Node) |
| **MCP3008** | SPI | 10-bit ADC for Potentiometer Feedback |
| **PCA9685** | I2C | 16-Channel PWM Driver for Servos |
| **MG996R / SG90** | PWM | Servo Actuators for Joints & Gripper |
| **Potentiometers** | Analog | Manual Input for HIL Control |

*(Optional: Add an image of your circuit diagram here)*

---

## 5. Project Structure 

```text
pi_bot/
├── robot_gui/           # PySide6 GUI Node & UI files
├── robot_description/   # URDF, Meshes, and Xacro files
├── robot_moveit_config/ # MoveIt2 configuration & SRDF
├── robot_hardware/      # Raspberry Pi Nodes (ADC/PWM drivers)
├── robot_interfaces/    # Custom Srv/Msg definitions (e.g. MoveToXYZ)
└── robot_bringup/       # Central Launch files
