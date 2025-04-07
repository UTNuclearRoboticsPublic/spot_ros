from launch import LaunchDescription, logging
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

import rclpy
import ros2node.api
import ros2component.api

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument('texture',
                            description='Whether to include color in pointclouds',
                            default_value='True'),

        DeclareLaunchArgument('hand_rgb', 
                            description='Whether to publish hand color pointcloud',
                            default_value='False'),

        DeclareLaunchArgument('hand_tof', 
                            description='Whether to publish hand mono pointcloud',
                            default_value='False'),

        DeclareLaunchArgument('frontleft',
                            description='Whether to publish front left camera pointcloud',
                            default_value='False'),

        DeclareLaunchArgument('frontright', 
                            description='Whether to publish front right camera pointcloud',
                            default_value='False'),

        DeclareLaunchArgument('left', 
                            description='Whether to publish left camera pointcloud',
                            default_value='False'),

        DeclareLaunchArgument('right', 
                            description='Whether to publish right camera pointcloud',
                            default_value='False'),

        DeclareLaunchArgument('back', 
                            description='Whether to publish rear camera pointcloud',
                            default_value='False'),

        DeclareLaunchArgument('all', 
                            description='Whether to publish all pointclouds',
                            default_value='False'),

        DeclareLaunchArgument('container',
                            description='Composition container in which to launch depth_image_proc nodes. If not set a new container will be created',
                            default_value='__new_container__'),

        DeclareLaunchArgument('use_sim_time',
                            default_value='False')
    ]

    container = LaunchConfiguration('container')

    # Opaque function is used to access launch args
    def addAllNodeDescriptions(context, *args, **kwargs):

        # Determine if to publish pointcloud with color data
        if LaunchConfigurationEquals('texture', 'True').evaluate(context):
            plugin_name = 'depth_image_proc::PointCloudXyzrgbNode'
            output_suffix = '_texture'
        else:
            plugin_name = 'depth_image_proc::PointCloudXyzNode'
            output_suffix = ''

        # Factory function for adding nodes to launched based on the Launch Configuration
        node_descriptions = []
        def addNodeDescription(base_ns: str, camera_name: str) -> None:
            if IfCondition(LaunchConfiguration(camera_name)).evaluate(context) or IfCondition(LaunchConfiguration('all')).evaluate(context):
                node_descriptions.append(
                    ComposableNode(
                        package='depth_image_proc',
                        plugin=plugin_name,
                        name=f'{camera_name}_cloud',
                        parameters=[
                            {'use_sim_time': LaunchConfiguration('use_sim_time')}
                        ],
                        remappings=[
                            ('points',                      f'/spot_pointclouds/{camera_name}_points{output_suffix}'),
                            ('image_rect',                  f'/{base_ns}/depth/{camera_name}/image'),
                            ('camera_info',                 f'/{base_ns}/depth/{camera_name}/camera_info'),
                            ('rgb/camera_info',             f'/{base_ns}/rgb/{camera_name}/camera_info'),
                            ('rgb/image_rect_color',        f'/{base_ns}/rgb/{camera_name}/image'),
                            ('depth_registered/image_rect', f'/{base_ns}/depth/{camera_name}/image'),
                        ],
                        extra_arguments=[{'use_intra_process_comms': False}],            
                    )
                )

        addNodeDescription('spot_image_server', 'frontleft' )
        addNodeDescription('spot_image_server', 'frontright')
        addNodeDescription('spot_image_server', 'left'      )
        addNodeDescription('spot_image_server', 'right'     )
        addNodeDescription('spot_image_server', 'back'      )
        addNodeDescription('spot_image_server', 'hand_tof'  )
        addNodeDescription('spot_image_server', 'hand_rgb'  )

        # If we need to create a new container, do so
        new_container = ComposableNodeContainer(
            condition=LaunchConfigurationEquals('container', '__new_container__'),
            name='depth_image_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=node_descriptions,
            output='screen'
        )

        # Otherwise load all the nodes into the specified existing container
        existing_container = LoadComposableNodes(
            condition=LaunchConfigurationNotEquals('container', '__new_container__'),
            target_container=container,
            composable_node_descriptions=node_descriptions,
        )

        # Deterine if the name that was passed is indeed an existing component container
        container_exists = False
        if LaunchConfigurationNotEquals('container', '__new_container__').evaluate(context):
            
            logging.get_logger('launch.user').info("\033[32mYou may safely ignore the warnings between these lines -----------------------------------\033[0m\n")
            rclpy.init()
            node = rclpy.create_node("find_container_nodes")
            node_names = ros2node.api.get_node_names(node=node)
            containers = ros2component.api.find_container_node_names(node=node, node_names=node_names)
            print("")
            logging.get_logger('launch.user').info("\033[32m------------------------------------------------------------------------------------------\033[0m")

            passed_container_name = container.perform(context)
            for container_name in containers:
                if container_name.name == passed_container_name or container_name.full_name == passed_container_name:
                    container_exists = True
                    break
        
            if not container_exists:
                container_message = LogInfo(msg=f"\033[33mWARNING: Passed container \"{passed_container_name}\" does not exist, your nodes will not launch until it is created\033[0m")
            else:
                container_message = LogInfo(msg=f"Found component container \"{passed_container_name}")
        
        else:
            container_message = LogInfo(msg="Creating a new component container called \"depth_image_proc_container\"")

        return new_container, existing_container, container_message


    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function = addAllNodeDescriptions)
    ])