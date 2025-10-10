# Author: Crasun Jans

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the configuration file
    config_file = os.path.join(
        get_package_share_directory('spot_apriltag'),
        'config',
        'spot_apriltag.yaml'
    )

    # Ensure the configuration file exists
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"The tag configuration file {config_file} was not found!")

    # Camera configurations
    camera_configs = [
        {
            'camera_names': ['frontleft', 'frontright', 'left', 'right'],
            'topic_prefix': '/spot_image_server/rgb/',
        }] # names are frontleft, frontright, left, right, back, and hand_rgb

    # Initialize empty list to hold nodes
    nodes = []

    # Generate nodes for each camera group using for loop
    for camera_config in camera_configs:
        for camera_name in camera_config['camera_names']:
            node = Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name=f'apriltag_node_{camera_name}',
                output='screen',
                remappings=[
                    ('image_rect', f'{camera_config["topic_prefix"]}{camera_name}/image'),
                    ('camera_info', f'{camera_config["topic_prefix"]}{camera_name}/camera_info')
                ],
                parameters=[config_file]
            )
            nodes.append(node)

    return LaunchDescription(nodes)
