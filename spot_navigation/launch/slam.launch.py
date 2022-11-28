import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument('map',
            default_value=os.path.join(
                FindPackageShare('spot_navigation').find('spot_navigation'),
                'map',
                'map.yaml'),
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    ]

    this_pkg_share = FindPackageShare('spot_navigation')
    params_file = PathJoinSubstitution([this_pkg_share, 'config', 'lidar_slam.yaml'])

    return LaunchDescription([
        *launch_args,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
                'params_file': params_file}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
        )
    ])
