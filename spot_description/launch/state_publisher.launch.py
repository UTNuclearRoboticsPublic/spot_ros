import os

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument('has_arm',
            description='Boolean. Include the Spot Arm.',
            choices=['True', 'False'],
            default_value='False'),
            
        DeclareLaunchArgument('has_eap',
            description='Boolean. Include the Enhanced Autonomy package (EAP)',
            choices=['True', 'False'],
            default_value='False')
    ]

    has_arm = LaunchConfiguration('has_arm')
    has_eap = LaunchConfiguration('has_eap')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    this_pkg_share = FindPackageShare('spot_description')
    
    # Build the URDF from the xacro, applying specified hardware accessories.
    xacro_path = PathJoinSubstitution([this_pkg_share, 'urdf', 'spot.urdf.xacro'])
    urdf_param = ParameterValue(
        Command(['xacro ', xacro_path, ' has_arm:=',has_arm, ' has_eap:=',has_eap]),
        value_type=str)

    return launch.LaunchDescription([
        *launch_args,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if True'),

        Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[{
                    'robot_description': urdf_param,
                    'use_sim_time': use_sim_time
               }]
            )
    ])
