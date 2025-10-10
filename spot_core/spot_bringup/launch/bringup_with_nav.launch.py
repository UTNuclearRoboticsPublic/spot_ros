from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
                            default_value="False"),
        

        DeclareLaunchArgument('auto_claim',
                            description='Claim ownership of the robot upon connection.',
                            default_value='False'),
        DeclareLaunchArgument('auto_power_on',
                            description='Power on the robot upon connection.',
                            default_value='False'),
        DeclareLaunchArgument('auto_stand',
                            description='Stand the robot upon connection.',
                            default_value='False')
    ]

    ## Robot bringup
    this_pkg = FindPackageShare('spot_bringup')
    bringup_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([this_pkg, 'launch', 'bringup.launch.py'])),
            launch_arguments=
                {'username':      LaunchConfiguration('username'),
                 'password':      LaunchConfiguration('password'),
                 'hostname':      LaunchConfiguration('hostname'),
                 'has_eap':       LaunchConfiguration('has_eap'),
                 'has_arm':       LaunchConfiguration('has_arm'),
                 'auto_claim':    LaunchConfiguration('auto_claim'),
                 'auto_power_on': LaunchConfiguration('auto_power_on'),
                 'auto_stand':    LaunchConfiguration('auto_stand')
                }.items()
            )

    ## Navigation
    spot_nav_pkg = FindPackageShare('spot_navigation')

    # Nav config file
    nav_config_file = PathJoinSubstitution([spot_nav_pkg, 'config', 'lidar.yaml'])

    # Include navigation
    nav_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([spot_nav_pkg, 'launch', 'navigation.launch.py'])),
            launch_arguments=
                {'config': nav_config_file}.items(),
            )

    ## Launch
    return LaunchDescription([
        *launch_args,
        bringup_include,
        nav_include
    ])
