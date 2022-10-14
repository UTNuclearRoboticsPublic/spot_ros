import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():

    launch_args = [
        launch.actions.DeclareLaunchArgument('has_arm', default_value='False'),
        launch.actions.DeclareLaunchArgument('has_velodyne', default_value='False')
    ]

    bringup_dir = FindPackageShare().find('spot_viz')
    return LaunchDescription([
        *launch_args,

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spot_description'),
                    'launch',
                    'description.launch.py'
                ])
            ),
            launch_arguments=[{'has_arm', LaunchConfiguration('has_arm')},
                              {'has_velodyne', LaunchConfiguration('has_velodyne')}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(bringup_dir,'rviz', 'robot.rviz')],
            on_exit=Shutdown()
        )
    ])
