from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    launch_args = [

        # Robot-specific configuration
        DeclareLaunchArgument('hostname',
                            default_value='no_value'),
        DeclareLaunchArgument('has_eap',
                            description="Robot includes the Extended Autonomy Package.",
                            default_value="True"),
        DeclareLaunchArgument('has_arm',
                            description='Robot includes the Spot Arm',
                            default_value="True"),
        DeclareLaunchArgument('has_eap_2',
                            description="Robot includes the Updated Extended Autonomy Package (EAP2).",
                            default_value="True"),
        DeclareLaunchArgument('has_realsense',
                            description='A realsense camera is mounted on the Spot Arm',
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

    has_arm       = LaunchConfiguration('has_arm')
    has_eap       = LaunchConfiguration('has_eap')
    has_eap_2     = LaunchConfiguration('has_eap_2')
    has_realsense = LaunchConfiguration('has_realsense')
    auto_claim    = LaunchConfiguration('auto_claim')
    auto_power_on = LaunchConfiguration('auto_power_on')
    auto_stand    = LaunchConfiguration('auto_stand')

    body_params = PathJoinSubstitution([FindPackageShare('spot_driver'), 'config', 'spot_ros.yaml'])
    arm_params  = PathJoinSubstitution([FindPackageShare('spot_manipulation_driver'), 'config', 'spot_arm.yaml'])

    ## Robot bringup
    driver_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('spot_driver'), 
                'launch', 
                'driver.launch.py'
            ])
        ),
        launch_arguments=
            {'hostname':      LaunchConfiguration('hostname'),
                'has_eap':       has_eap,
                'has_arm':       has_arm,
                'has_eap_2':     has_eap_2,
                'auto_claim':    auto_claim,
                'auto_power_on': auto_power_on,
                'auto_stand':    auto_stand,
            }.items()
    )

    combined_driver = Node(
        condition=IfCondition(has_arm),
        package='spot_manipulation_driver',
        executable='combined_driver_node',
        parameters=[
            {'hostname': LaunchConfiguration('hostname'),
            'has_eap_2': has_eap_2},
            body_params,
            arm_params
        ]
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
                        'has_eap': has_eap,
                        'has_eap_2': has_eap_2}.items()
    )

    # Realsense
    realsense_include = IncludeLaunchDescription(
        condition=IfCondition(has_realsense),
        launch_description_source = PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'pointcloud.enable': 'True',
            'clip_distance': '2.0',
            'rgb_camera.profile': '424x240x15',
            'depth_module.profile': '424x240x15'
        }.items()
    )

    # Teleop 
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node'
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='spot_teleop_node',
        parameters=[
            {'require_enable_button': True},
            {'enable_button': 4},
            {'axis_linear.x': 1},
            {'axis_linear.y': 0},
            {'scale_linear.x': 0.5},
            {'scale_linear.y': 0.5},
            {'axis_angular.yaw': 2},
            {'scale_angular.yaw': 0.5}
        ],
        remappings=[
            ('cmd_vel', '/spot_driver/cmd_vel')
        ]
    )

    # Body Teleop Commands
    spot_joy_node = Node(
        package='spot_bringup',
        executable='spot_joy',
        name='spot_joy_node',
    )

    # Arm Teleop Commands
    spot_arm_joy_include = IncludeLaunchDescription(
        launch_description_source = PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('spot_manipulation_driver'), 
                'launch', 
                'arm_teleop_joy.launch.py'
            ])
        ),
        condition=IfCondition(has_arm)
    )

    ## Launch
    return LaunchDescription([
        *launch_args,
        driver_include,
        combined_driver,
        state_publisher_include,
        realsense_include,
        joy_node,
        teleop_twist_joy_node,
        spot_joy_node,
        spot_arm_joy_include
    ])
