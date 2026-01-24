from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    """
    Launches the ROS 2 control stack for the robot.

    Intent:
    - Load the robot description (URDF)
    - Start ros2_control controller manager
    - Spawn required controllers in the correct order
    """

    robot_description = ParameterValue(
        Command(
        [
            "xacro ",
            os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro")
        ]), 
        value_type=str,
    )

    """
    robot_state_publisher:
    - Publishes TF based on joint states
    - Required for RViz, MoveIt, and visualization
    - use_sim_time parameter only important when using Gazebo or IsaacSim
    """
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time" : True}]
    )

    """
    Controller manager (ros2_control_node):
    - Central manager for all controllers
    - Loads hardware interfaces from the URDF
    - Reads controller configuration from YAML
    """
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description},
                     os.path.join(get_package_share_directory("robot_controller"),
                                  "config",
                                  "robot_controllers.yaml")]
    )

    """
    Joint State Broadcaster:

    Intent:
    - Publishes /joint_states -> needed for Rviz, MoveIt and PCA9685 servo controller
    - Required before any other controller
    - Must be spawned first
    """
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Loads the JointTrajectoryController for the arm
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Loads the GripperActionController for the gripper
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    """
    Startup order matters:
    - robot_state_publisher can start immediately
    - controller_manager must start before spawners
    - joint_state_broadcaster should be spawned before the other controllers
    """
    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner
    ])