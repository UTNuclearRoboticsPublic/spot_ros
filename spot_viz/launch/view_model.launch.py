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
        launch.actions.DeclareLaunchArgument('has_arm',
            description='Boolean. Include the Spot Arm.',
            choices=['True', 'False'],
            default_value='False'),

        launch.actions.DeclareLaunchArgument('has_eap',
            description='Boolean. Include the Enhanced Autonomy package (EAP).',
            choices=['True', 'False'],
            default_value='False')
    ]

    has_arm = LaunchConfiguration('has_arm')
    has_eap = LaunchConfiguration('has_eap')
    this_pkg_share = FindPackageShare('spot_viz')

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
                    'state_publisher.launch.py'
                ])
            ),
            launch_arguments=[{'has_arm', has_arm},
                              {'has_eap', has_eap}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([this_pkg_share, 'rviz', 'model.rviz'])],
            on_exit=Shutdown(reason='RViz exited.')
        )
    ])
