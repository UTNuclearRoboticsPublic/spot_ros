from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.parameters_type import ParameterDescription, ParameterFile
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  
  launch_args = [
    DeclareLaunchArgument("username", default_value=TextSubstitution(text="dummy_username")),
    DeclareLaunchArgument("password", default_value=TextSubstitution(text="dummy_password")),
    DeclareLaunchArgument("hostname", default_value=TextSubstitution(text="192.168.50.3"))
  ]

  nodes = [
    Node(
        package='spot_driver',
        namespace='spot',
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
                               value_type=str)
        ],
        remappings=[
          ('tf','/tf'),
          ('joint_states','/joint_states')
        ],
        on_exit=Shutdown()
    ),

    Node(
        package='twist_mux',
        namespace='spot',
        executable='twist_mux',
        name='twist_mux',
        parameters=[
          ParameterFile(PathJoinSubstitution([
                    FindPackageShare('spot_driver'),
                    'config',
                    'twist_mux.yaml"'
                ]))
        ],
        remappings=[
          ('cmd_vel_out','spot/cmd_vel')
        ]
    )
  ]

  return LaunchDescription([
      *launch_args,
      *nodes
  ])