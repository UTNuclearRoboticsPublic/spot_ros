from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
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
                            default_value="True"),
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
                            default_value='False')
    ]

    has_eap = LaunchConfiguration('has_eap')
    has_arm = LaunchConfiguration('has_arm')

    camera_fps = 10.0

    ## Robot bringup
    driver_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('spot_driver'), 'launch', 'driver.launch.py'])),
            launch_arguments=
                {'username':      LaunchConfiguration('username'),
                 'password':      LaunchConfiguration('password'),
                 'hostname':      LaunchConfiguration('hostname'),
                 'has_eap':       has_eap,
                 'has_arm':       has_arm,
                 'auto_claim':    LaunchConfiguration('auto_claim'),
                 'auto_power_on': LaunchConfiguration('auto_power_on'),
                 'auto_stand':    LaunchConfiguration('auto_stand'),
                 'camera_fps':    TextSubstitution(text=str(camera_fps))
                }.items()
            )

    # State publisher
    state_publisher_include = IncludeLaunchDescription(
      launch_description_source = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
              FindPackageShare('spot_description'),
              'launch',
              'state_publisher.launch.py'
        ])
      ),
      launch_arguments={'has_arm': has_arm,
                        'has_eap': has_eap}.items()
    )

    realsense_include = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'pointcloud.enable': 'True',
            'clip_distance': '2.0'
        }.items()
    )

    ## Launch
    return LaunchDescription([
        *launch_args,
        driver_include,
        state_publisher_include,
        realsense_include
    ])
