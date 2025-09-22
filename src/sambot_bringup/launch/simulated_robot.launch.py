import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument("use_slam", default_value="false")

    sambot_description_dir = get_package_share_directory("sambot_description")
    world_path = os.path.join(sambot_description_dir, "worlds", "house.world")

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sambot_description"),
            "launch",
            "gazebo.launch.py",
        ),
        launch_arguments={"world": world_path}.items(),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sambot_controller"),
            "launch",
            "controller.launch.py",
        )
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sambot_localization"),
            "launch",
            "global_localization.launch.py",
        ),
        condition=UnlessCondition(use_slam),
    )
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sambot_mapping"), "launch", "slam.launch.py"
        ),
        condition=IfCondition(use_slam),
    )
    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("sambot_localization"),
                "rviz",
                "global_localization.rviz",
            ),
        ],
        output="screen",
        condition=UnlessCondition(use_slam),
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("sambot_mapping"), "rviz", "slam.rviz"
            ),
        ],
        output="screen",
        condition=IfCondition(use_slam),
    )

    return LaunchDescription(
        [
            use_slam_arg,
            gazebo,
            controller,
            localization,
            slam,
            rviz_localization,
            rviz_slam,
        ]
    )
