import math
from launch import LaunchDescription
from launch.conditions import LaunchConfigurationNotEquals
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, IfElseSubstitution, NotEqualsSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument('config',
            description="Filepath for navigation configuration. See navigation2 package documentation.",
            default_value=PathJoinSubstitution([FindPackageShare('spot_navigation'), 'config', 'spot.yaml'])),

        DeclareLaunchArgument('initial_pose_file',
            description='An initial pose to pass to the AMCL which overwrites that in the config file',
            default_value="{}"),

        DeclareLaunchArgument('map',
            default_value=PathJoinSubstitution([
                FindPackageShare('spot_navigation'),
                'map',
                'new_map.yaml']),
            description='Full path to map file to load. Set to "none" to omit this and use an external map server'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'cloud_in',
            default_value='/velodyne_points',
            description='Pointcloud to use for localization')
    ]

    laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        parameters=[
            {"angle_min": -math.pi},
            {"angle_max":  math.pi},
            {"angle_increment": math.radians(1.0)},
            {"target_frame": "base_footprint"},
            {"min_height": 0.40},
            {"max_height": 1.5}
        ],
        remappings=[
            ("/spot_nav/cloud_in", LaunchConfiguration('cloud_in')),
        ]
    )

    map_server = Node(
        condition=LaunchConfigurationNotEquals('map', 'none'),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': LaunchConfiguration('map')},
            {'frame_id': 'spot_nav/map'}
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            LaunchConfiguration('config'),
            LaunchConfiguration('initial_pose_file')
        ]
    )

    # Create a lifecycle manager to start AMCL and the Map Server
    lifecycle_nodes = IfElseSubstitution(
        NotEqualsSubstitution(LaunchConfiguration('map'), 'none'),
        if_value='[map_server, amcl]',
        else_value='[amcl]'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
    )

    return LaunchDescription([
        *launch_args,
        PushRosNamespace("spot_nav"),
        amcl,
        map_server,
        laserscan_node,
        lifecycle_manager
    ])
