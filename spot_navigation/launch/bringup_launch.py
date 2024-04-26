from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    nav_include = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/spot_driver/cmd_vel'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
                ),
                launch_arguments={
                    "params_file": PathJoinSubstitution([FindPackageShare("spot_navigation"), "config", "spot.yaml"])
                }.items()
            )
        ]
    )

    return LaunchDescription([
        nav_include
    ])
