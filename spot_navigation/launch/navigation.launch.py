import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([FindPackageShare('spot_navigation'), 'config', 'spot.yaml']),
            description='YAML file with all relevant navigation configuration parameters'
        )
    ]

    config_file = LaunchConfiguration('config_file')

    lifecycle_nodes = ['planner_server',
                       'controller_server',
                       'recoveries_server',
                       'bt_navigator']
    
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config_file],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('cmd_vel', '/spot_driver/cmd_vel')
        ]
    )

    recoveries_server = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('cmd_vel', '/spot_driver/cmd_vel')
        ]
    )

    bt_server = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[config_file],
        remappings=[],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )


    return LaunchDescription([
        *launch_args,
        planner_server,
        controller_server, 
        recoveries_server,
        bt_server,
        lifecycle_manager
    ])
