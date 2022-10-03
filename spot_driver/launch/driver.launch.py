from typing import Text

from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

from launch_ros.actions import Node
from launch_ros.parameters_type import ParameterDescription

def generate_launch_description():
  
  launch_args = [
    DeclareLaunchArgument("username", default_value=TextSubstitution(text="dummy_username")),
    DeclareLaunchArgument("password", default_value=TextSubstitution(text="dummy_password")),
    DeclareLaunchArgument("hostname", default_value=TextSubstitution(text="192.168.50.3")),
    DeclareLaunchArgument("auto_claim", default_value=TextSubstitution(text="False")),
    DeclareLaunchArgument("auto_power_on", default_value=TextSubstitution(text="False")),
    DeclareLaunchArgument("auto_stand", default_value=TextSubstitution(text="False"))
  ]

  nodes = [
    Node(
        package='spot_driver',
        executable='driver',
        name='spot_driver',
        parameters=[
          ParameterDescription(name='username',
                               value=LaunchConfiguration('username'),
                               value_type=Text),
          ParameterDescription(name='password',
                               value=LaunchConfiguration('password'),
                               value_type=Text),
          ParameterDescription(name='hostname',
                               value=LaunchConfiguration('hostname'),
                               value_type=Text),
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

  return LaunchDescription([
      *launch_args,
      *nodes
  ])