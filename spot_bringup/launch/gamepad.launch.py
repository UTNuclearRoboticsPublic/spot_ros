from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from spot_description.get_accessories import get_accessories_from_env

def generate_launch_description():

    spot_accessories_dict = get_accessories_from_env()

    launch_args = [
        DeclareLaunchArgument('has_arm',
            description='Robot includes the Spot Arm',
            default_value=spot_accessories_dict.get('has_arm', 'False')),
        DeclareLaunchArgument('controller_configuration',
            description='Name of the controller configuration to use for teleoperation',
            choices=['Logitech', 'Dualsense5'],
            default_value='Dualsense5'),
    ]

    # Teleop
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        parameters=[{'autorepeat_rate': 50.0}]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='spot_teleop_node',
        parameters=[
            {'require_enable_button': True},
            {'enable_button': 4},
            {'axis_linear.x': 1},
            {'axis_linear.y': 0},
            {'scale_linear.x': 2.0},
            {'scale_linear.y': 2.0},
            {'axis_angular.yaw': 2},
            {'scale_angular.yaw': 1.5}
        ],
        remappings=[
            ('cmd_vel', '/controller/cmd_vel')
        ]
    )

    # Body Teleop Commands
    spot_joy_node = Node(
        package='spot_bringup',
        executable='spot_joy',
        name='spot_joy_node',
        parameters=[
            {'controller': LaunchConfiguration('controller_configuration')},
            {'dock_id': LaunchConfiguration('dock_id')}
        ]
    )

    # Arm Teleop Commands
    spot_arm_joy_include = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('spot_manipulation_driver'),
                'launch',
                'arm_teleop_joy.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('has_arm'))
    )

    return LaunchDescription([
        *launch_args,
        joy_node,
        teleop_twist_joy_node,
        spot_joy_node,
        spot_arm_joy_include,
    ])