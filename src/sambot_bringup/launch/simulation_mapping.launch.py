from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to the launch files
    sambot_description_launch = os.path.join(
        get_package_share_directory('sambot_description'),
        'launch',
        'gazebo.launch.py'
    )

    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    sambot_controller_launch = os.path.join(
        get_package_share_directory('sambot_controller'),
        'launch',
        'controller.launch.py'
    )

    mapper_params_file = os.path.join(
        os.getcwd(),  # Assuming you run from workspace root
        # 'src',
        'sambot_controller',
        'config',
        'mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sambot_description_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch),
            launch_arguments={'params_file': mapper_params_file}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sambot_controller_launch)
        ),
    ])

# Helper function for ROS 2 package share directory