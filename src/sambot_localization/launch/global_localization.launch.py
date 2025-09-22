from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():

    map_name_arg = DeclareLaunchArgument("map_name", default_value="small_house")
    map_name = LaunchConfiguration("map_name")
    lifecycle_nodes=["map_server"]
    map_path = PathJoinSubstitution(
        [
            get_package_share_directory("sambot_mapping"),
            "maps",
            map_name,
            "map.yaml",
        ]
    )

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_path}],
    )
    
    nav2_lifecycle_manager=Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names":lifecycle_nodes},
            {"autostart":True}
        ]
    )

    return LaunchDescription([
        map_name_arg,
        nav2_map_server,
        nav2_lifecycle_manager
    ])
