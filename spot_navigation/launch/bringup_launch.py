from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace, SetRemap

def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config',
        description='Full path to the navigation configuration file',
        default_value=PathJoinSubstitution([FindPackageShare("spot_navigation"), "config", "spot.yaml"])
    )

    pointcloud_arg = DeclareLaunchArgument(
        'cloud_in',
        description='The topic on which to look for pointcloud data. Default "/velodyne_points"',
        default_value='/velodyne_points'
    )
    
    nav_include = GroupAction(
        actions=[
            PushRosNamespace("spot_nav"),
            SetRemap(src='cmd_vel'   , dst='/nav_stack/cmd_vel'),
            SetRemap(src='/tf'       , dst='/tf'),
            SetRemap(src='/tf_static', dst='/tf_static'),
            SetRemap(src='/velodyne_points', dst=LaunchConfiguration('cloud_in')),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
                ),
                launch_arguments={
                    "params_file": LaunchConfiguration('config')
                }.items()
            )
        ]
    )

    return LaunchDescription([
        config_arg,
        pointcloud_arg,
        nav_include,
    ])
