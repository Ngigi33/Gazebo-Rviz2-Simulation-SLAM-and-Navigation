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

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("sambot_mapping"), "config", "slam_toolbox.yaml"
        ),
    )
    slam_config = LaunchConfiguration("slam_config")
    lifecycle_nodes = ["map_saver_server"]

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config],
    )

    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"fresh_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65},
        ],
    )
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"node_names": lifecycle_nodes}, {"autostart": True}],
    )

    return LaunchDescription(
        [
            slam_config_arg,
            nav2_map_saver,
            slam_toolbox,
            nav2_lifecycle_manager,
        ]
    )
