from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    """
    Launch file for visualizing the robot model.

    This launch file:
    - Loads the robot URDF via XACRO
    - Publishes the robot state (TF tree)
    - Starts the robot GUI
    - Launches RViz with a predefined configuration

    Gazebo or physics simulation is intentionally NOT included here.
    This launch file is focused on visualization and interaction only.
    """
    # Launch argument allowing to override the robot model path 
    # ROS2 best practice for reusable launch files
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro"),
        description="Absolute path to the robot URDF file"
        )
    
    # Generate the robot_description parameter by executing xacro at runtime
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    # Publishes TF transforms based on the URDF and incoming joint states
    # Required for RViz visualization and kinematic calculations
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # GUI node for interacting with the robot
    robot_gui_node = Node(
        package="robot_gui",
        executable="robot_advanced_gui_node",
        output="screen"
    )

    # RViz node with a predefined configuration 
    # Config can be found and changed in the rviz folder of this package
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("robot_description"), "rviz", "display.rviz")]
    )
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        robot_gui_node,
        rviz_node
    ])