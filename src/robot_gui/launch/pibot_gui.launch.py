import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    """
    Central launch file for the PiBot robot system.

    This launch file integrates:
    - Robot State Publisher (TF Tree)
    - ROS 2 Control (Hardware interface & Controller Manager)
    - Controller Spawners (Arm, Gripper, Joint States)
    - MoveIt 2 (Move Group planning & execution)
    - Visualization (RViz2)
    - Custom Application Nodes (GUI, FK calculations, XYZ Services)
    """
    # --- 1. Path Resolution and Launch Arguments ---
    # Resolve package paths for configuration files
    moveit_pkg_path = get_package_share_directory("robot_moveit")
    description_pkg_path = get_package_share_directory("robot_description")
    controller_pkg_path = get_package_share_directory("robot_controller")

    # Launch argument to toggle between simulation time and real time
    # Set to 'False' by default to avoid timeouts when no simulator (Gazebo) is running
    is_sim_arg = DeclareLaunchArgument("is_sim", default_value="False")
    is_sim = LaunchConfiguration("is_sim")

    # --- 2. Robot Description (URDF) ---
    # Load URDF via Xacro - shared by State Publisher, Controller Manager, and MoveIt
    robot_description_content = Command(
        [
            "xacro ",
            os.path.join(description_pkg_path, "urdf", "robot.urdf.xacro")
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # --- 3. MoveIt 2 Configuration ---
    # Build MoveIt configuration using the MoveItConfigsBuilder helper
    moveit_config = (
        MoveItConfigsBuilder("pibot", package_name="robot_moveit")
        .robot_description(file_path=os.path.join(description_pkg_path, "urdf", "robot.urdf.xacro"))
        .robot_description_semantic(file_path=os.path.join(moveit_pkg_path, "config", "robot.srdf"))
        .trajectory_execution(file_path=os.path.join(moveit_pkg_path, "config", "moveit_controllers.yaml"))
        .joint_limits(file_path=os.path.join(moveit_pkg_path, "config", "joint_limits.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # --- 4. Node Definitions ---

    # Robot State Publisher: Publishes the static/dynamic transforms (TF) of the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": is_sim}]
    )

    # Controller Manager: The core of ros2_control managing hardware and controllers
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            os.path.join(controller_pkg_path, "config", "robot_controllers.yaml"),
            {"use_sim_time": is_sim}
        ]
    )

    # Joint State Broadcaster: Publishes /joint_states from hardware to TF
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    # Arm Controller: Manages the Joint Trajectory for the main robotic arm
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"]
    )

    # Gripper Controller: Manages the Gripper action (Open/Close)
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"]
    )

    # MoveIt Move Group: Provides the planning scene and motion planning services
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True}
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    # RViz2: Interactive 3D visualization tailored for MoveIt planning
    rviz_config = os.path.join(moveit_pkg_path, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": is_sim}
        ]
    )

    # --- 5. Application-Specific Nodes ---
    # Robot Advanced GUI: Custom control interface for the robot
    robot_gui_node = Node(
        package="robot_gui",
        executable="robot_advanced_gui_node",
        output="screen",
        parameters=[{"use_sim_time": is_sim}]
    )

    # Move XYZ Service: Service-based interface for cartesian movements
    move_it_services_node = Node(
        package="move_it_services",
        executable="move_to_xyz_service",
        output="screen",
        parameters=[{"use_sim_time": is_sim}]
    )

    # Robot FK: Node for calculating Forward Kinematics
    robot_fk_node = Node(
        package="robot_fk",
        executable="robot_fk",
        output="screen",
        parameters=[{"use_sim_time": is_sim}]
    )

    # --- 6. Final Launch Description ---
    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        move_group_node,
        rviz_node,
        robot_gui_node,
        move_it_services_node,
        robot_fk_node
    ])