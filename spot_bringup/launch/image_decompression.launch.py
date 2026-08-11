from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    compressed_image_sources = ['front', 'left', 'right', 'frontleft', 'frontright', 'hand_rgb']
    nodes = [
        Node(
            package='image_transport',
            executable='republish',
            remappings=(
                ('in/compressed', f'/spot_image_server/rgb/{image_source}/image/compressed'),
                ('out', f'/spot_image_server/rgb/{image_source}/image')
            ),
            arguments=[
                'compressed',
                'raw'
            ]
        )
        for image_source in compressed_image_sources
    ]

    return LaunchDescription(nodes)