import os

import launch
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():

    launch_args = [
        launch.actions.DeclareLaunchArgument('has_arm', default_value='False'),
        launch.actions.DeclareLaunchArgument('has_velodyne', default_value='False')
    ]

    urdf_mappings = {}
    if LaunchConfiguration('has_arm') == 'True':
        urdf_mappings['has_arm'] = 'true'
    if LaunchConfiguration('has_velodyne') == 'True':
        urdf_mappings['has_velodyne'] = 'true'
    
    pkg_share = FindPackageShare().find('spot_description')
    filepath = os.path.join(pkg_share, 'urdf', 'spot.urdf.xacro')
    robot_desc = xacro.process_file(filepath,
                                    mappings=urdf_mappings
                                    ).toprettyxml(indent='  ')

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[{'robot_description': robot_desc}]
                                 )

    return launch.LaunchDescription([
        *launch_args,
        rsp
    ])
