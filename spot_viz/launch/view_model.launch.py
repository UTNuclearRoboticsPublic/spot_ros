from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():

    bringup_dir = get_package_share_directory('spot_viz')
    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            namespace='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            remappings=[
                ('/joint_state_publisher_gui/joint_states', '/joint_states'),
                ('/joint_state_publisher_gui/robot_description', '/robot_description'),
                ('/joint_state_publisher_gui/parameter_events', '/parameter_events')
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spot_description'),
                    'launch',
                    'description.launch.py'
                ])
            ),
            launch_arguments=[{'has_arm', LaunchConfiguration('show_arm', default='False')}]
        ),
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(bringup_dir,'rviz', 'robot.rviz"')],
            on_exit=Shutdown()
        )
    ])
