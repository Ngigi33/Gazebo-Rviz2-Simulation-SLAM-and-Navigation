from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from os import pathsep
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    sambot_description = get_package_share_directory("sambot_description")
    sambot_description_prefix = get_package_prefix("sambot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("sambot_description"),
            "urdf",
            "robot.urdf.xacro",
        ),
        description="Absolute path to robot URDF file",
    )

    model_path = os.path.join(sambot_description, "models")
    model_path += pathsep + os.path.join(sambot_description_prefix,"share")

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution(
        [
            sambot_description,
            "worlds",
            PythonExpression(
                expression=["'", LaunchConfiguration("world_name"), "'", "+'.world'"]
            ),
        ]
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzserver.launch.py",
            )
        ),
        launch_arguments={"world": world_path}.items(),
    )
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py",
            )
        )
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "robot", "-topic", "robot_description"],
        parameters=[{"timeout": 60}],
        output="screen",
    )

    return LaunchDescription(
        [
            env_var,
            # gazebo,
            model_arg,
            world_name_arg,
            start_gazebo_server,
            start_gazebo_client,
            robot_state_publisher,
            spawn_robot,
        ]
    )
