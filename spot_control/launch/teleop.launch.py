from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameters_type import ParameterFile
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():

    has_joystick = DeclareLaunchArgument("joystick", default_value=TextSubstitution(text="False")),

    nodes = [
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                prefix = 'xterm -e',
                on_exit=Shutdown(),
                remappings=[
                    ('cmd_vel', 'keyboard/cmd_vel')
                ]
            ),
            Node(
                package='twist_mux',
                executable='twist_mux',
                name='twist_mux',
                parameters=[
                ParameterFile(PathJoinSubstitution([
                            FindPackageShare('spot_control'),
                            'config',
                            'twist_mux.yaml'
                        ]))
                ],
                remappings=[
                    ('cmd_vel_out', 'spot_driver/cmd_vel')
                ]
            )
    ]

    if has_joystick:
        nodes.append(
            Node(
                package='joy',
                executable='joy_node',
                name='joystick_driver'
            )
        )
        nodes.append(
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy'
            )
        )

    return LaunchDescription([
        *nodes
    ])
