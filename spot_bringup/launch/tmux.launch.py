import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, OrSubstitution, AndSubstitution, NotSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter

def generate_launch_description():

    twist_mux_config = PathJoinSubstitution([
        FindPackageShare('spot_bringup'),
        'config',
        'twist_mux.yaml'
    ])

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        name='twist_mux',
        remappings=[('/cmd_vel_out', '/spot_driver/cmd_vel')],
        parameters=[
            twist_mux_config]
    )

    ## Launch
    return LaunchDescription([
        *launch_args,
        driver_include,
        combined_driver,
        state_publisher_include,
        realsense_include,
        joy_node,
        teleop_twist_joy_node,
        spot_joy_node,
        spot_arm_joy_include,
        velodyne_include,
        twist_mux
    ])
