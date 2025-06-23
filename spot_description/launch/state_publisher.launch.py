import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument('has_arm',
            description='Boolean. Include the Spot Arm.',
            choices=['True', 'False'],
            default_value='False'),
            
        DeclareLaunchArgument('has_eap',
            description='Boolean. Include the Enhanced Autonomy package (EAP)',
            choices=['True', 'False'],
            default_value='False'),

        DeclareLaunchArgument('has_eap_2',
            description='Boolean. Include the Updated Enhanced Autonomy package (EAP2)',
            choices=['True', 'False'],
            default_value='False'),

        DeclareLaunchArgument('has_rl_kit',
            description='Boolean. Include the RL Research Kit mounting set',
            choices=['True', 'False'],
            default_value='False'),

        DeclareLaunchArgument('has_realsense',
            description='Boolean. Include an arm mounted Realsense D435',
            choices=['True', 'False'],
            default_value='False'),

        DeclareLaunchArgument('has_cam_payload',
            description='Boolean. Include the CAM payload',
            choices=['True', 'False'],
            default_value='False'),

        DeclareLaunchArgument('kinematic_model',
            description='The kinematic model to use for the Spot description',
            choices=['none', 'body_assist', 'mobile_manipulation'],
            default_value='none'),

        DeclareLaunchArgument('use_proprietary_meshes',
            description='Whether to use proprietary meshes',
            choices=['True', 'False'],
            default_value='True'),

        DeclareLaunchArgument(
            'proprietary_pkg',
            description='Name of the package containing proprietary meshes',
            default_value='spot_proprietary_description'),
        
        DeclareLaunchArgument(
            'proprietary_mesh_format',
            description='File extension format for proprietary mesh files',
            default_value='dae',
            choices=['dae', 'stl']
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Build the URDF from the xacro, applying specified hardware accessories.
    launch_arg_names = ['has_arm', 'has_eap', 'has_eap_2', 'has_rl_kit', 'has_realsense', 'has_cam_payload', 'kinematic_model', 'use_proprietary_meshes', 'proprietary_pkg', 'proprietary_mesh_format']
    xacro_command_args = [elem for arg_name in launch_arg_names for elem in (f' {arg_name}:=', LaunchConfiguration(arg_name))]
    xacro_path = PathJoinSubstitution([FindPackageShare('spot_description'), 'urdf', 'spot.urdf.xacro'])
    urdf_param = ParameterValue(Command(['xacro ', xacro_path, *xacro_command_args]), value_type=str)

    return launch.LaunchDescription([
        *launch_args,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if True'),

        Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[{
                    'robot_description': urdf_param,
                    'use_sim_time': use_sim_time
               }],
               remappings=[
                   ('joint_states', '/spot_driver/joint_states'),
                   ('robot_description', '/spot_driver/robot_description')
               ]
            )
    ])
