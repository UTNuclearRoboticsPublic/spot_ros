from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition

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
    DeclareLaunchArgument('has_eap_2',
                          description="Robot includes the Updated Extended Autonomy Package(EAP2).",
                          default_value="False"),
    DeclareLaunchArgument('has_arm',
                          description='Robot includes the Spot Arm',
                          default_value="False"),
    DeclareLaunchArgument('has_cam_payload',
                          description='Robot includes the CAM payload',
                          default_value='False'),
    DeclareLaunchArgument('kinematic_model',
                          description='The kinematic model to use for the Spot description',
                          choices=['none', 'body_assist', 'mobile_manipulation'],
                          default_value='none'),
    DeclareLaunchArgument('auto_claim',
                          description='Claim ownership of the robot upon connection.',
                          default_value='False'),
    DeclareLaunchArgument('auto_power_on',
                          description='Power on the robot upon connection.',
                          default_value='False'),
    DeclareLaunchArgument('auto_stand',
                          description='Stand the robot upon connection.',
                          default_value='False'),
    DeclareLaunchArgument('publish_images',
                          description='Specify whether to publish (colored) images',
                          default_value='False'),
    DeclareLaunchArgument('publish_depth_images',
                          description='Specify whether to publish depth images',
                          default_value='False'),
    DeclareLaunchArgument('launch_pointcloud_service',
                        description='Launch the robot pointcloud service instead of interfacing with the LiDAR directly',
                        default_value='False'),
    DeclareLaunchArgument('robot_state_update_rate',
                          description='The update rate of the robot state (including TF) in Hz',
                          default_value='10.0')
  ]

  has_arm = LaunchConfiguration('has_arm')
  has_eap = LaunchConfiguration('has_eap')

  body_params = PathJoinSubstitution([FindPackageShare('spot_driver'), 'config', 'spot_ros.yaml'])


  # If the robot has an arm, we do not launch the pure body driver
  nodes = [
    Node(
      package='spot_driver',
      executable='driver',
      name='spot_driver',
      condition=UnlessCondition(has_arm),
      parameters=[
        body_params,
        ParameterDescription(name='hostname',
                             value=LaunchConfiguration('hostname'),
                             value_type=str),
        ParameterDescription(name='has_eap',
                             value=has_eap,
                             value_type=bool),
        ParameterDescription(name='has_eap_2',
                             value=LaunchConfiguration('has_eap_2'),
                             value_type=bool),
        ParameterDescription(name='has_cam_payload',
                             value=LaunchConfiguration('has_cam_payload'),
                             value_type=bool),
        ParameterDescription(name='kinematic_model',
                             value=LaunchConfiguration('kinematic_model'),
                             value_type=str),
        ParameterDescription(name='auto_claim',
                             value=LaunchConfiguration('auto_claim'),
                             value_type=bool),
        ParameterDescription(name='auto_power_on',
                             value=LaunchConfiguration('auto_power_on'),
                             value_type=bool),
        ParameterDescription(name='auto_stand',
                             value=LaunchConfiguration('auto_stand'),
                             value_type=bool),
        ParameterDescription(name='publish_images',
                             value=LaunchConfiguration('publish_images'),
                             value_type=bool),
        ParameterDescription(name='publish_depth_images',
                             value=LaunchConfiguration('publish_depth_images'),
                             value_type=bool),
        ParameterDescription(name='launch_pointcloud_service',
                             value=LaunchConfiguration('launch_pointcloud_service'),
                             value_type=bool),
        ParameterDescription(name='rates.status.robot_state',
                             value=LaunchConfiguration('robot_state_update_rate'),
                             value_type=float)
      ],
      output='screen',
      on_exit=Shutdown()
    )
  ]

  return LaunchDescription([
      *launch_args,
      *nodes,
  ])
