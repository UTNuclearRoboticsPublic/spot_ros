import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_dir = LaunchConfiguration('map')

    return LaunchDescription([

        DeclareLaunchArgument('config',
            description="Filepath for navigation configuration. See navigation2 package documentation."),

        DeclareLaunchArgument('map',
            default_value=os.path.join(
                FindPackageShare('spot_navigation').find('spot_navigation'),
                'map',
                'map.yaml'),
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch','bringup_launch.py'])),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': LaunchConfiguration('config')}.items(),
        )
    ])
