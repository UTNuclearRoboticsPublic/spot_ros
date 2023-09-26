from launch import LaunchDescription
from launch.conditions import IfCondition

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameters_type import ParameterDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
  
  launch_args = [

    # Robot-specific configuration
    DeclareLaunchArgument('username',
                          default_value='no_value'),
    DeclareLaunchArgument('password',
                          default_value='no_value'),
    DeclareLaunchArgument('hostname',
                          default_value='no_value'),
    DeclareLaunchArgument('has_eap',
                          description="Robot includes the Extended Autonomy Package.",
                          default_value="False"),
    DeclareLaunchArgument('has_arm',
                          description='Robot includes the Spot Arm',
                          default_value="True"),
    

    DeclareLaunchArgument('auto_claim',
                          description='Claim ownership of the robot upon connection.',
                          default_value='False'),
    DeclareLaunchArgument('auto_power_on',
                          description='Power on the robot upon connection.',
                          default_value='False'),
    DeclareLaunchArgument('auto_stand',
                          description='Stand the robot upon connection.',
                          default_value='False'),

    DeclareLaunchArgument('camera_fps',
                          description='The desired camera frames-per-second.',
                          default_value='1.0'),
    DeclareLaunchArgument('robot_state_update_rate',
                          description='The update rate of the robot state (including TF) in Hz',
                          default_value='10.0')
  ]

  has_eap = LaunchConfiguration('has_eap')
  has_arm = LaunchConfiguration('has_arm')
  camera_fps = LaunchConfiguration('camera_fps')

  # Two versions of this node. See the IfCondition and UnlessCondition.
  nodes = [
    Node(
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
                             value=has_eap,
                             value_type=bool),
        ParameterDescription(name='auto_claim',
                             value=LaunchConfiguration('auto_claim'),
                             value_type=bool),
        ParameterDescription(name='auto_power_on',
                             value=LaunchConfiguration('auto_power_on'),
                             value_type=bool),
        ParameterDescription(name='auto_stand',
                             value=LaunchConfiguration('auto_stand'),
                             value_type=bool),
        ParameterDescription(name='rates.sensors.front_image',
                             value=camera_fps,
                             value_type=float),
        ParameterDescription(name='rates.sensors.side_image',
                             value=camera_fps,
                             value_type=float),
        ParameterDescription(name='rates.sensors.rear_image',
                             value=camera_fps,
                             value_type=float),
        ParameterDescription(name='rates.sensors.hand_image',
                             value=camera_fps,
                             value_type=float),
        ParameterDescription(name='rates.status.robot_state',
                             value=LaunchConfiguration('robot_state_update_rate'),
                             value_type=float)
      ],
      output='screen',
      on_exit=Shutdown()
    )
  ]

  includes = [
    IncludeLaunchDescription(
      condition=IfCondition(has_eap),
      launch_description_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('velodyne'), 'launch',
                              'velodyne-all-nodes-VLP16-composed-launch.py'])
      ))
  ]

  return LaunchDescription([
      *launch_args,
      *nodes,
      *includes
  ])