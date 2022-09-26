import os

import launch
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import Parameter

import xacro

def generate_launch_description():
    has_arm = LaunchConfiguration('has_arm', default='False')

    
    if has_arm:
        xacro_file = 'spot_with_arm.urdf.xacro'
    else:
        xacro_file = 'spot.urdf.xacro'
    
    pkg_share = FindPackageShare('spot_description').find('spot_description')
    filepath = os.path.join(pkg_share, 'urdf', xacro_file)
    robot_desc = xacro.process_file(filepath).toprettyxml(indent='  ')

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[{'robot_description': robot_desc}]
                                 )

    return launch.LaunchDescription([rsp])
