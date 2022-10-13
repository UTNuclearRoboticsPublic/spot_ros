import os

import launch
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():
    has_arm_arg = launch.actions.DeclareLaunchArgument('has_arm', default_value='False')
    has_velodyne_arg = launch.actions.DeclareLaunchArgument('has_velodyne', default_value='False')

    has_arm = LaunchConfiguration('has_arm')
    has_velodyne = LaunchConfiguration('has_velodyne')
    
    xacro_file = 'spot.urdf.xacro'
    
    pkg_share = FindPackageShare('spot_description').find('spot_description')
    filepath = os.path.join(pkg_share, 'urdf', xacro_file)
    robot_desc = xacro.process_file(filepath,
                                    mappings={'has_arm': has_arm_arg,
                                              'has_velodyne': has_velodyne}
                                    ).toprettyxml(indent='  ')

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[{'robot_description': robot_desc}]
                                 )

    return launch.LaunchDescription([rsp])
