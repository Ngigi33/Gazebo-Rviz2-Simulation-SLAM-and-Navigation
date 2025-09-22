from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import GroupAction


def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.0425")

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation", default_value="0.245"
    )

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "sambot_diffdrive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    cmd_republisher=Node(
        package="sambot_controller",
        executable="cmd_vel_republisher"
    )

    odom_republisher=Node(
        package="sambot_controller",
        executable="odom_republisher"
    )

    return LaunchDescription(
        [
            # wheel_radius_arg,
            # wheel_separation_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            cmd_republisher,
            odom_republisher
        ]
    )
