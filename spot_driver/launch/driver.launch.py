from typing import Text

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.parameters_type import ParameterDescription, ParameterFile
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  cfg_file = DeclareLaunchArgument('robot_config_file', description='Parameter file for robot connection and startup.')
  
  launch_args = [
    # DeclareLaunchArgument("username", default_value=TextSubstitution(text="dummy_username")),
    # DeclareLaunchArgument("password", default_value=TextSubstitution(text="dummy_password")),
    # DeclareLaunchArgument("hostname", default_value=TextSubstitution(text="192.168.50.3")),

    cfg_file,
    DeclareLaunchArgument("auto_claim", default_value=TextSubstitution(text="False")),
    DeclareLaunchArgument("auto_power_on", default_value=TextSubstitution(text="False")),
    DeclareLaunchArgument("auto_stand", default_value=TextSubstitution(text="False")),
    DeclareLaunchArgument("has_eap", description='True if the robot includes the Extended Autonomy Package',
                          default_value="False")
  ]

  nodes = [
    Node(
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
  ]

  includes = []
  if LaunchConfiguration('has_eap'):
    includes.append(
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          PathJoinSubstitution([FindPackageShare('velodyne'), 'launch',
                               'velodyne-all-nodes-VLP16-composed-launch.py'])
        )))

  return LaunchDescription([
      *launch_args,
      *nodes,
      *includes
  ])