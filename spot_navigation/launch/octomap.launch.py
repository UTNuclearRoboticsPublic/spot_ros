from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'cloud_in',
            default_value='velodyne_points',
            description='The topic on which to listen for the pointcloud data'
        ),

        DeclareLaunchArgument(
            'config_file',
            default_value='octomap_params.yaml',
            description='The file within the /spot_navigation/config folder with which to look for params'
        ),
    ]

    octomap_server = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server_node",
        parameters=[
            PathJoinSubstitution([FindPackageShare('spot_navigation'), 'config', LaunchConfiguration('config_file')])
        ],
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_in'))
        ]
    )

    return LaunchDescription([
        *launch_args,
        octomap_server
    ])

