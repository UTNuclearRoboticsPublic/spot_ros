import os
from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node, ComposableNodeContainer
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, OrSubstitution, AndSubstitution, NotSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import PushRosNamespace, SetRemap
from spot_description.get_accessories import get_accessories_from_env

def generate_launch_description():

    spot_accessories_dict = get_accessories_from_env()

    launch_args = [

        # Robot-specific configuration
        DeclareLaunchArgument('hostname',
                            default_value='no_value'),
        DeclareLaunchArgument('velodyne_ip',
                            default_value='192.168.1.201'),
        DeclareLaunchArgument('spot_namespace',
                            default_value=''),
        DeclareLaunchArgument('image_config',
                            default_value=PathJoinSubstitution([
                                FindPackageShare("spot_driver"),
                                "config",
                                "image_service_only.yaml"
                            ]),
                            description="Path to image config YAML. By default we don't publish images"),
        DeclareLaunchArgument('dock_id',
                            default_value='520'),

        # Accessories
        DeclareLaunchArgument('has_eap',
                            description="Robot includes the Extended Autonomy Package",
                            default_value=spot_accessories_dict.get('has_eap', 'False')),
        DeclareLaunchArgument('has_arm',
                            description='Robot includes the Spot Arm',
                            default_value=spot_accessories_dict.get('has_arm', 'False')),
        DeclareLaunchArgument('has_eap_2',
                            description="Robot includes the Updated Extended Autonomy Package (EAP2)",
                            default_value=spot_accessories_dict.get('has_eap_2', 'False')),
        DeclareLaunchArgument('kinematic_model',
                            description='The kinematic model to use for the Spot description',
                            choices=['none', 'body_assist', 'mobile_manipulation'],
                            default_value='none'),
        DeclareLaunchArgument('has_rl_kit',
                            description="Robot includes the Reinforcement Learning Research Kit mounting setup",
                            default_value=spot_accessories_dict.get('has_rl_kit', 'False')),
        DeclareLaunchArgument('has_cam_payload',
                            description="Robot includes the CAM payload.",
                            default_value=spot_accessories_dict.get('has_cam_payload', 'False')),
        DeclareLaunchArgument('has_realsense',
                            description='A realsense camera is mounted on the Spot Arm',
                            default_value=spot_accessories_dict.get('has_realsense', 'False')),

        # Startup actions
        DeclareLaunchArgument('auto_claim',
                            description='Claim ownership of the robot upon connection.',
                            default_value='False'),
        DeclareLaunchArgument('auto_power_on',
                            description='Power on the robot upon connection.',
                            default_value='False'),
        DeclareLaunchArgument('auto_stand',
                            description='Stand the robot upon connection.',
                            default_value='False'),
        DeclareLaunchArgument('launch_pointcloud_service',
                            description='Launch the robot pointcloud service instead of interfacing with the LiDAR directly',
                            default_value='False'),
        DeclareLaunchArgument('launch_velodyne',
                            description='Launch the velodyne driver regardless of accessory settings',
                            default_value='True'),

        # Other configurations
        DeclareLaunchArgument('manipulation_action_namespace',
                            description='Namespace for the manipulation action servers. Temporary fix until remppaing is added to action servers (https://github.com/ros2/rcl/pull/1170)',
                            default_value=''),
        DeclareLaunchArgument('controller_configuration',
                            description='Name of the controller configuration to use for teleoperation',
                            choices=['Logitech', 'Dualsense5'],
                            default_value='Dualsense5'),
        DeclareLaunchArgument('data_capture_mode',
                            description='Whether to published received joint trajectories on corresponding action server goal topics',
                            default_value='False'),
        DeclareLaunchArgument('use_proprietary_meshes',
            description='Whether to use proprietary meshes',
            default_value='False'),
        DeclareLaunchArgument(
            'proprietary_pkg',
            description='Name of the package containing proprietary meshes',
            default_value='spot_proprietary_description'),
        DeclareLaunchArgument(
            'proprietary_mesh_format',
            description='File extension format for proprietary mesh files',
            default_value='dae',
            choices=['dae', 'stl', 'obj']
        ),
    ]

    dock_id         = LaunchConfiguration('dock_id')
    has_arm         = LaunchConfiguration('has_arm')
    has_eap         = LaunchConfiguration('has_eap')
    has_eap_2       = LaunchConfiguration('has_eap_2')
    kinematic_model = LaunchConfiguration('kinematic_model')
    has_rl_kit      = LaunchConfiguration('has_rl_kit')
    has_realsense   = LaunchConfiguration('has_realsense')
    has_cam_payload = LaunchConfiguration('has_cam_payload')
    use_proprietary_meshes = LaunchConfiguration('use_proprietary_meshes')
    proprietary_pkg = LaunchConfiguration('proprietary_pkg')
    proprietary_mesh_format = LaunchConfiguration('proprietary_mesh_format')
    auto_claim      = LaunchConfiguration('auto_claim')
    auto_power_on   = LaunchConfiguration('auto_power_on')
    auto_stand      = LaunchConfiguration('auto_stand')
    launch_pointcloud_service = LaunchConfiguration('launch_pointcloud_service')

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
                'has_eap':         has_eap,
                'has_arm':         has_arm,
                'has_eap_2':       has_eap_2,
                'kinematic_model': kinematic_model,
                'has_rl_kit':      has_rl_kit,
                'has_cam_payload': has_cam_payload,
                'use_proprietary_meshes':   use_proprietary_meshes,
                'proprietary_pkg':          proprietary_pkg,
                'proprietary_mesh_format':  proprietary_mesh_format,
                'auto_claim':      auto_claim,
                'auto_power_on':   auto_power_on,
                'auto_stand':      auto_stand,
                'launch_pointcloud_service': launch_pointcloud_service
            }.items()
    )

    combined_driver = Node(
        condition=IfCondition(has_arm),
        package='spot_manipulation_driver',
        executable='combined_driver_node',
        parameters=[
            {'hostname': LaunchConfiguration('hostname'),
            'has_eap':         has_eap,
            'has_arm':         has_arm,
            'has_eap_2':       has_eap_2,
            'kinematic_model': kinematic_model,
            'has_rl_kit':      has_rl_kit,
            'has_cam_payload': has_cam_payload,
            'use_proprietary_meshes':   use_proprietary_meshes,
            'proprietary_pkg':          proprietary_pkg,
            'proprietary_mesh_format':  proprietary_mesh_format,
            'auto_claim':      auto_claim,
            'auto_power_on':   auto_power_on,
            'auto_stand':      auto_stand,
            'launch_pointcloud_service': launch_pointcloud_service,
            'action_namespace': LaunchConfiguration('manipulation_action_namespace'),
            'data_capture_mode': LaunchConfiguration('data_capture_mode'),
            },
            body_params,
            arm_params
        ]
    )

    image_publisher_include = Node(
        package='spot_driver',
        executable='image_server',
        parameters=[
            {'hostname': LaunchConfiguration('hostname')},
            LaunchConfiguration('image_config')
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
                        'has_eap_2': has_eap_2,
                        'kinematic_model': kinematic_model,
                        'has_rl_kit': has_rl_kit,
                        'has_cam_payload': has_cam_payload,
                        'use_proprietary_meshes':   use_proprietary_meshes,
                        'proprietary_pkg':          proprietary_pkg,
                        'proprietary_mesh_format':  proprietary_mesh_format,}.items()
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
        name='joy_node',
        parameters=[{'autorepeat_rate': 50.0}]
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
            {'scale_linear.x': 2.0},
            {'scale_linear.y': 2.0},
            {'axis_angular.yaw': 2},
            {'scale_angular.yaw': 1.5}
        ],
        remappings=[
            ('cmd_vel', '/controller/cmd_vel')
        ]
    )

    # Body Teleop Commands
    spot_joy_node = Node(
        package='spot_bringup',
        executable='spot_joy',
        name='spot_joy_node',
        parameters=[
            {'controller': LaunchConfiguration('controller_configuration')},
            {'dock_id': LaunchConfiguration('dock_id')}
        ]
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

    velodyne_include = GroupAction(
        # Only launch velodyne if we have the EAP or the EAP2 and we're not using the pointcloud service
        condition=IfCondition(
            OrSubstitution(
                LaunchConfiguration('launch_velodyne'),
                OrSubstitution(
                    has_eap,
                    AndSubstitution(
                        has_eap_2,
                        NotSubstitution(LaunchConfiguration('launch_pointcloud_service'))
                    )
                )
            )
        ),
        # Run velodyne nodes manually so we have access to parameter reassignment
        actions=[
            PushRosNamespace(LaunchConfiguration('spot_namespace')),
            ComposableNodeContainer(
                name='velodyne_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='velodyne_driver',
                        plugin='velodyne_driver::VelodyneDriver',
                        name='velodyne_driver_node',
                        parameters=[
                            PathJoinSubstitution([FindPackageShare('spot_bringup'), 'config', 'velodyne_config.yaml']),
                            {'device_ip': LaunchConfiguration('velodyne_ip')}
                        ]
                    ),
                    ComposableNode(
                        package='velodyne_pointcloud',
                        plugin='velodyne_pointcloud::Transform',
                        name='velodyne_transform_node',
                        parameters=[
                            PathJoinSubstitution([FindPackageShare('spot_bringup'), 'config', 'velodyne_config.yaml']),
                            {'calibration': PathJoinSubstitution([FindPackageShare('velodyne_pointcloud'), 'params', 'VLP16db.yaml'])}
                        ],
                        remappings=[
                            ('velodyne_points', 'unfiltered_velodyne_points')
                        ]
                    ),
                    ComposableNode(
                        name='spot_pointcloud_filter_component',
                        package='spot_navigation',
                        plugin='spot_navigation::PointcloudFilterComponent',
                        remappings=[
                            ('cloud_in', 'unfiltered_velodyne_points'),
                            ('cloud_out','velodyne_points')
                        ]
                    )
                ]
            )
        ],
    )

    twist_mux_config = PathJoinSubstitution([
        FindPackageShare('spot_driver'),
        'config',
        'twist_mux.yaml'
    ])

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        name='twist_mux',
        remappings=[('/cmd_vel_out', '/spot_driver/cmd_vel')],
        parameters=[twist_mux_config]
    )

    ## Launch
    return LaunchDescription([
        *launch_args,
        driver_include,
        combined_driver,
        image_publisher_include,
        state_publisher_include,
        realsense_include,
        joy_node,
        teleop_twist_joy_node,
        spot_joy_node,
        spot_arm_joy_include,
        velodyne_include,
        twist_mux
    ])
