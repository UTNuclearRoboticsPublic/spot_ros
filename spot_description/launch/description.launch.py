import os

import launch
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

import launch_ros.actions
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    launch_args = [
        launch.actions.DeclareLaunchArgument('has_arm',
            description='Boolean. Include the Spot Arm.',
            choices=['True', 'False'],
            default_value='False'),
            
        launch.actions.DeclareLaunchArgument('has_eap',
            description='Boolean. Include the Enhanced Autonomy package (EAP)',
            choices=['True', 'False'],
            default_value='False')
    ]

    has_arm = LaunchConfiguration('has_arm')
    has_eap = LaunchConfiguration('has_eap')
    this_pkg_share = FindPackageShare('spot_description')
    
    # Build the URDF from the xacro, applying specified hardware accessories.
    xacro_path = PathJoinSubstitution([this_pkg_share, 'urdf', 'spot.urdf.xacro'])
    urdf_param = ParameterValue(
        Command(['xacro ', xacro_path, ' has_arm:=',has_arm, ' has_eap:=',has_eap]),
        value_type=str)

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[{
                                    'robot_description': urdf_param
                                  }]
                                 )

    return launch.LaunchDescription([
        *launch_args,
        rsp
    ])
