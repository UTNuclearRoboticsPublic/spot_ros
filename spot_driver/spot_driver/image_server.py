#!/usr/bin/env python3

from __future__ import annotations
from logging import Logger
from asyncio import Future, InvalidStateError

import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster

from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request, UnknownImageSourceError, SourceDataError, UnsetStatusError, ImageDataError
from bosdyn.client.exceptions import RpcError
from bosdyn.client.frame_helpers import get_a_tform_b, BODY_FRAME_NAME, HAND_FRAME_NAME

from spot_msgs.srv import GetImages
from spot_driver.image_server_parameters import spot_driver_parameters

from .async_queries import AsyncImageService
from .ros_helpers import getImageMsg, populateTransformStamped, TimestampToMsg
from .spot_body_wrapper import SpotLeaseManager
from .type_hint_helpers import *

class SpotImageServer(Node):
    """ Inner class for managing camera publishing """
    class CameraPub():
        def __init__(self, parent: SpotImageServer, namespace: str):
            self.parent = parent
            self.image_pub = parent.create_publisher(Image, '~/' + namespace + '/image', 1)
            self.info_pub = parent.create_publisher(CameraInfo, '~/' + namespace+'/camera_info', 1)
            self.lease_manager = parent.lease_manager

        def process_data(self, data: ImageResponseProto):
            if self.image_pub.get_subscription_count() > 0:
                image_msg, camera_info_msg, _ = getImageMsg(data, self.lease_manager)
                self.image_pub.publish(image_msg)
                self.info_pub.publish(camera_info_msg)

    def __init__(self):
        super().__init__('spot_image_server')
        self.get_logger().info('Starting Spot Image Server')

        # Initialize ROS side
        self.camera_pubs: dict[str, self.CameraPub] = {}

        parameter_listener = spot_driver_parameters.ParamListener(self)
        self.params = parameter_listener.get_params()

        self.get_logger().info(f'Creating image services for the following sources: {", ".join(self.params.image_sources)}')

        self.get_image_service = self.create_service(GetImages, '~/get_images', self.get_image_callback)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Connect to robot
        self.lease_manager = SpotLeaseManager()
        self.lease_manager.setLogger(self.get_logger())
        if not self.lease_manager.connect(self.params.hostname):
            raise RuntimeError('Aborting spot_image_server bringup')

        logger_py = Logger(self.get_name())

        try:
            self.image_client = self.lease_manager.robot.ensure_client(ImageClient.default_service_name)
        except Exception as e:
            raise RuntimeError(f'Unable to create image client: {e}')

        # Initialize Boston Dynamics image services
        self.image_requests: dict[str, ImageRequestProto] = {}
        self.continuous_publish_tasks: list[AsyncImageService] = []

        self.get_logger().info('Creating publishers:')
        for image_source in self.params.image_sources:
            
            if image_source.startswith('hand') and not self.lease_manager.robot.has_arm():
                self.get_logger().warn(f'Robot does not have arm. Skipping image source {image_source}')
                continue

            rgb_source, depth_source = self.resolve_source_name(image_source)

            rgb_pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8 if image_source != 'hand_tof' else None
            self.image_requests[rgb_source] = build_image_request(rgb_source, image_format=image_pb2.Image.FORMAT_RAW, pixel_format=rgb_pixel_format)
            self.image_requests[depth_source] = build_image_request(depth_source, image_format=image_pb2.Image.FORMAT_RAW)

            rgb_rate = self.params.rates.get_entry(image_source).rgb
            depth_rate = self.params.rates.get_entry(image_source).depth

            if rgb_rate > 0:
                self.camera_pubs[rgb_source] = self.CameraPub(self, 'rgb/' + image_source)
                self.continuous_publish_tasks.append(AsyncImageService(
                    client=self.image_client,
                    logger=logger_py,
                    rate=rgb_rate,
                    callback=self.image_callback, 
                    image_requests=[self.image_requests[rgb_source]]
                ))
                self.get_logger().info(f'Publishing to rgb/{image_source} at {rgb_rate} Hz')

            if depth_rate > 0:
                self.camera_pubs[depth_source] = (self.CameraPub(self, 'depth/' + image_source))
                self.continuous_publish_tasks.append(AsyncImageService(
                    client=self.image_client,
                    logger=logger_py,
                    rate=depth_rate,
                    callback=self.image_callback, 
                    image_requests=[self.image_requests[depth_source]]
                ))
                self.get_logger().info(f'Publishing to depth/{image_source} at {depth_rate} Hz')

        # Update the tf tree with the transforms to the calibrated body cameras
        self.publish_static_transforms()

        # Start a loop to constantly check for image messages from the robot at 20Hz
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.update_timer = self.create_timer(0.05, lambda: [task.update() for task in self.continuous_publish_tasks], callback_group=self.timer_group)

        self.get_logger().info(f'Spot Image Server online')

    def resolve_source_name(self, parameter_source: str) -> tuple[str, str]:
        if parameter_source.startswith('hand'):
            if parameter_source.endswith('tof'):
                rgb_source = 'hand_image'
                depth_source = 'hand_depth'

            elif parameter_source.endswith('rgb'):
                rgb_source = 'hand_color_image'
                depth_source = 'hand_depth_in_hand_color_frame'

            else:
                raise RuntimeError(f'Unknown hand source passed to image server: {parameter_source}')

        else:
            rgb_source = parameter_source + '_fisheye_image'
            depth_source = parameter_source + '_depth'

        return rgb_source, depth_source
    
    def publish_static_transforms(self):
        # Publish body static transforms
        body_requests = [req for (name, req) in self.image_requests.items() if 'hand' not in name]
        if len(body_requests):
            body_image_responses = self.image_client.get_image(body_requests)
            for image_response in body_image_responses:
                _, _, tf_message = getImageMsg(image_response, self.lease_manager)
                self.tf_broadcaster.sendTransform([tform for tform in tf_message.transforms if tform.child_frame_id != 'odom' and tform.child_frame_id != 'vision'])

        # Publish hand static transforms
        hand_requests = [req for (name, req) in self.image_requests.items() if 'hand' in name]
        if len(hand_requests):
            hand_image_responses = self.image_client.get_image(hand_requests)
            body_snapshot = self.lease_manager._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
            body_tform_hand = get_a_tform_b(body_snapshot, BODY_FRAME_NAME, HAND_FRAME_NAME)
            for image_response in hand_image_responses:
                body_tform_image_frame = get_a_tform_b(image_response.shot.transforms_snapshot, BODY_FRAME_NAME, image_response.shot.frame_name_image_sensor)
                hand_tform_image_frame = body_tform_hand.inverse() * body_tform_image_frame
                transform_stamped = populateTransformStamped(
                    time=TimestampToMsg(self.lease_manager.robotToLocalTime(image_response.shot.acquisition_time)),
                    parent_frame='arm0_hand',
                    child_frame=image_response.shot.frame_name_image_sensor,
                    transform=hand_tform_image_frame
                )
                self.tf_broadcaster.sendTransform(transform_stamped)
        
    def image_callback(self, response_future: Future):
        try:
            response: ImageResponseProto = response_future.result()[0]
            self.camera_pubs[response.source.name].process_data(response)
        except InvalidStateError as e:
            # This path is taken if the image proto has not been returned yet
            # We do nothing and continue waiting for it to arrive
            pass
        except IndexError as e:
            self.get_logger().warn(f'Image future returned with no images to process: {e}')
        except Exception as e:
            self.get_logger().warn(f'Unknown error in image callback: {e}')

    def get_image_callback(self, req: GetImages.Request, resp: GetImages.Response) -> GetImages.Response:
        resp.success = False

        # Make sure the provided sources were registered on startup
        for source in req.sources:
            if source not in self.image_requests.keys():
                self.get_logger().warn(f'Provided image source {source} does not exist')
                return resp

        # Request the images from the robot 
        try:
            image_responses = self.image_client.get_image([self.image_requests[source] for source in req.sources])
            for response in image_responses:
                if response.status != image_pb2.ImageResponse.Status.STATUS_OK:
                    self.get_logger().warn(f'Unable to retrieve image from {response.source.name}')
                    return resp

                image_msg, camera_info, _ = getImageMsg(response, self.lease_manager)
                resp.images.append(image_msg)
                resp.camera_infos.append(camera_info)

            resp.success = True

        except RpcError as e:
            self.get_logger().warn(f'Error to communicating with robot: {e}')
        except UnknownImageSourceError as e:
            self.get_logger().warn(f'Provided image source does not exist: {e}')
        except (SourceDataError, UnsetStatusError, ImageDataError) as e:
            self.get_logger().warn(f'Unable to retrive image from robot: {e}')

        return resp

def main():
    rclpy.init()
    try:
        image_server = SpotImageServer()
    except Exception as e:
        rclpy.logging.get_logger('spot_image_server').error(f'{e}')
        exit(1)
    mt_exec = MultiThreadedExecutor(num_threads=2)
    mt_exec.add_node(image_server)
    mt_exec.spin()

    rclpy.shutdown()
