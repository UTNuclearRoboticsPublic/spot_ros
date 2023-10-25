import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument('config',
            description="Filepath for navigation configuration. See navigation2 package documentation.",
            default_value=PathJoinSubstitution([FindPackageShare('spot_navigation'), 'config', 'spot.yaml'])),

        DeclareLaunchArgument('map',
            default_value=PathJoinSubstitution([
                FindPackageShare('spot_navigation'),
                'map',
                'ahg.yaml']),
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')
    ]

    laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan_node",
        parameters=[
            {"angle_min": -math.pi},
            {"angle_max":  math.pi},
            {"angle_increment": math.radians(1.0)},
            {"target_frame": "gpe"},
            {"min_height": 0.20},
            {"max_height": 1.5}
        ],
        remappings=[
            ("cloud_in", "/velodyne_points"),
            ("scan", "/spot/pointcloud_scan")
        ]
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {'yaml_filename': PathJoinSubstitution([FindPackageShare('spot_navigation'), 'map', 'ahg.yaml'])}
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            LaunchConfiguration('config')
        ]
    )

    # Create a lifecycle manager to start AMCL and the Map Server
    lifecycle_nodes = ['map_server', 'amcl']
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
        amcl,
        map_server,
        laserscan_node,
        lifecycle_manager
    ])
