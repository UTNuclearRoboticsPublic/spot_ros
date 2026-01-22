import launch
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('has_arm',
            description='Boolean. Include the Spot Arm.',
            default_value='False')
    ]
    
    return launch.LaunchDescription([
        *launch_args,
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'body', '--child-frame-id', 'head']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'head', '--child-frame-id', 'frontleft', '--x', '0.415', '--y', '0.036', '--z', '0.024', '--qx', '0.144', '--qy', '0.809', '--qz', '-0.222', '--qw', '0.524']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'frontleft', '--child-frame-id', 'frontleft_fisheye', '--x', '0.076']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'head', '--child-frame-id', 'frontright', '--x', '0.415', '--y', '-0.037', '--z', '0.023', '--qx', '-0.146', '--qy', ' 0.811', '--qz', '0.224', '--qw', '0.521']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'frontright', '--child-frame-id', 'frontright_fisheye', '--x', '0.077']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'head', '--child-frame-id', 'left', '--x', '-0.161', '--y', '0.111', '--z', '0.037', '--qx', '-0.795', '--qy', '0.000', '--qz', '0.000', '--qw', '0.607']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'left', '--child-frame-id', 'left_fisheye', '--x', '0.077']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'head', '--child-frame-id', 'right', '--x', '-0.171', '--y', '-0.109', '--z', '0.036', '--qx', '0.789', '--qy', '0.000', '--qz', '0.000', '--qw', '0.614']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'right', '--child-frame-id', 'right_fisheye', '--x', '0.077']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'head', '--child-frame-id', 'back', '--x', '-0.417', '--y', '-0.039', '--z', '0.009', '--qx', '0.562', '--qy', '0.563', '--qz', '-0.430', '--qw', '-0.427']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'back', '--child-frame-id', 'back_fisheye', '--x', '0.077']
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'arm0_hand', '--child-frame-id', 'hand_depth_sensor', '--x', '-0.058', '--z', '-0.004', '--qw', '0.749', '--qy', '0.663'],
            condition=IfCondition(LaunchConfiguration('has_arm'))
        ),
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher', 
            arguments=['--frame-id', 'arm0_hand', '--child-frame-id', 'hand_color_image_sensor', '--x', '-0.058', '--y', '-0.020', '--z', '0.025', '--qx', '-0.459', '--qy', '0.459', '--qz', '-0.538', '--qw', '0.538'],
            condition=IfCondition(LaunchConfiguration('has_arm'))
        )
    ])