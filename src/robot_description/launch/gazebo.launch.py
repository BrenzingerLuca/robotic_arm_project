import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    robot_description_dir = get_package_share_directory("robot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro"),
        description="Absolute path to the robot URDF file"
        )
    
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(robot_description_dir).parent.resolve())
        ]
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

#Hier bei parameters topic zu meinem aendern 
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time" : True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ros_gz_sim"), 
                "launch"
            ), "/gz_sim.launch.py"]
        ),
        launch_arguments=[
            ("gz_args", ["-v 4 -r empty.sdf"])
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "Armi2.0"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge", 
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])