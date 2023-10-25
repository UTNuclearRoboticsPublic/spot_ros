import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    ]

    this_pkg_share = FindPackageShare('spot_navigation')
    params_file = PathJoinSubstitution([this_pkg_share, 'config', 'lidar_slam.yaml'])

    return LaunchDescription([
        *launch_args,

        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan_node",
            parameters=[
                {"angle_min": -math.pi},
                {"angle_max":  math.pi},
                {"angle_increment": math.radians(1.0)},
                {"target_frame": "gpe"},
                {"min_height": 0.20},
                {"max_height": 1.5}
            ],
            remappings=[
                ("cloud_in", "/velodyne_points"),
                ("scan", "/spot/pointcloud_scan")
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
                launch_arguments={
                    'slam_params_file': params_file,
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items()
        )
    ])
