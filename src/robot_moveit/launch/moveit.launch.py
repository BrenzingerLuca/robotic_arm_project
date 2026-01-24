from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for starting MoveIt for the PiBot robot.

    This launch file:
    - Loads the robot URDF and SRDF
    - Configures joint limits, controllers, and kinematics
    - Starts the MoveIt move_group node
    - Launches RViz with a MoveIt-specific configuration

    Gazebo or physics simulation is NOT started here.
    The `is_sim` argument is provided for future integration
    with simulation environments.
    """

    # Launch argument to switch between real-time and simulated time
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    # Resolve package paths of the packages containing the necessary .yaml files
    moveit_pkg_path = get_package_share_directory("robot_moveit")
    description_pkg_path = get_package_share_directory("robot_description")


    # Build the MoveIt configuration using MoveItConfigsBuilder
    moveit_config = (
        MoveItConfigsBuilder("pibot", package_name="robot_moveit")
        # Load the robot URDF
        .robot_description(file_path=os.path.join(description_pkg_path, "urdf", "robot.urdf.xacro"))
        # Load the robot SRDF (planning groups, end effectors, collisions)
        .robot_description_semantic(file_path=os.path.join(moveit_pkg_path, "config", "robot.srdf"))
        # Load the configuration of the trajectory execution and controllers
        .trajectory_execution(file_path=os.path.join(moveit_pkg_path, "config", "moveit_controllers.yaml"))
        # Load joint limits
        .joint_limits(file_path=os.path.join(moveit_pkg_path, "config", "joint_limits.yaml"))
        # Select planning pipelines
        # OMPL is used here; additional pipelines can be added later in rviz
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Core MoveIt node responsible for planning and execution
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": LaunchConfiguration("is_sim")}, {"publish_robot_description_semantic": True}],
        # Enable informative logging
        arguments=["--ros-args", "--log-level", "info"]
    )

    # RViz configuration tailored for MoveIt visualization 
    rviz_config = os.path.join(get_package_share_directory("robot_moveit"), "config", "moveit.rviz")

    # RViz node for interactive planning and visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        # Explicitly provide robot and MoveIt parameters to RViz
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits]
    )

    return LaunchDescription([
        is_sim_arg,
        move_group_node,
        rviz_node
    ])
    