from typing import Text

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.parameters_type import ParameterDescription, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  
  launch_args = [
    DeclareLaunchArgument('robot_config_file',
                          description='Parameter file for robot connection and startup.',
                          default_value='no_file_given'),

    # These are ignored if 'robot_config_file' is given
    # In that case these params come from that file instead
    DeclareLaunchArgument("username", default_value=TextSubstitution(text='dummy_username')),
    DeclareLaunchArgument("password", default_value=TextSubstitution(text='dummy_password')),
    DeclareLaunchArgument("hostname", default_value=TextSubstitution(text='192.168.50.3')),
    DeclareLaunchArgument('has_eap',
                          description='True if the robot includes the Extended Autonomy Package',
                          default_value="False"),
    DeclareLaunchArgument('has_arm',
                          description='True if the robot includes the Spot Arm',
                          default_value="False"),
    
    # these are NOT ignored if 'robot_config_file' is given
    DeclareLaunchArgument('auto_claim', default_value='False'),
    DeclareLaunchArgument('auto_power_on', default_value='False'),
    DeclareLaunchArgument('auto_stand', default_value='False')
  ]

  if LaunchConfiguration('robot_config_file') == 'no_file_given':
    driver_node = Node(
        package='spot_driver',
        executable='driver',
        name='spot_driver',
        parameters=[
          ParameterDescription(name='username',
                               value=LaunchConfiguration('username'),
                               value_type=str),
          ParameterDescription(name='password',
                               value=LaunchConfiguration('password'),
                               value_type=str),
          ParameterDescription(name='hostname',
                               value=LaunchConfiguration('hostname'),
                               value_type=str),
          ParameterDescription(name='has_eap',
                               value=LaunchConfiguration('has_eap'),
                               value_type=bool),
          ParameterDescription(name='auto_claim',
                               value=LaunchConfiguration('auto_claim'),
                               value_type=bool),
          ParameterDescription(name='auto_power_on',
                               value=LaunchConfiguration('auto_power_on'),
                               value_type=bool),
          ParameterDescription(name='auto_stand',
                               value=LaunchConfiguration('auto_stand'),
                               value_type=bool)
        ],
        on_exit=Shutdown()
    )
  else:
    driver_node = Node(
        package='spot_driver',
        executable='driver',
        name='spot_driver',
        parameters=[
          ParameterFile(LaunchConfiguration('robot_config_file')),
          ParameterDescription(name='auto_claim',
                               value=LaunchConfiguration('auto_claim'),
                               value_type=bool),
          ParameterDescription(name='auto_power_on',
                               value=LaunchConfiguration('auto_power_on'),
                               value_type=bool),
          ParameterDescription(name='auto_stand',
                               value=LaunchConfiguration('auto_stand'),
                               value_type=bool)
        ],
        on_exit=Shutdown()
    )

  includes = []
  if LaunchConfiguration('has_eap'):
    includes.append(
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          PathJoinSubstitution([FindPackageShare('velodyne'), 'launch',
                               'velodyne-all-nodes-VLP16-composed-launch.py'])
        )))

  includes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spot_description'),
                    'launch',
                    'description.launch.py'
                ])
            ),
            launch_arguments=[{'has_arm', LaunchConfiguration('has_arm')},
                              {'has_velodyne', LaunchConfiguration('has_eap')}]
        ))

  return LaunchDescription([
      *launch_args,
      driver_node,
      *includes
  ])