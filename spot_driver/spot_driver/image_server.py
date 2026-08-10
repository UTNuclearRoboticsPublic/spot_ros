#!/usr/bin/env python3

from __future__ import annotations
from asyncio import Future, InvalidStateError

import time
import threading
from collections import defaultdict

import rclpy
import rclpy.logging
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.time import Time
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from tf2_ros import StaticTransformBroadcaster

from bosdyn.api import image_pb2
from bosdyn.client import InvalidRequestError
from bosdyn.client.image import ImageClient, build_image_request, UnknownImageSourceError, SourceDataError, UnsetStatusError, ImageDataError
from bosdyn.client.exceptions import RpcError
from bosdyn.client.frame_helpers import get_a_tform_b, BODY_FRAME_NAME, HAND_FRAME_NAME

from spot_msgs.srv import GetImages
from spot_driver.image_server_parameters import spot_driver_parameters

from .ros_helpers import getImageMsg, populateTransformStamped, TimestampToMsg, UnsupportedImageFormatError
from .spot_body_wrapper import SpotLeaseManager
from .type_hint_helpers import *

def new_fps_bucket() -> dict:
    return {'count': 0, 'bytes': 0, 'lat_sum': 0.0, 'lat_max': 0.0}

""" Class for managing camera publishing """
class CameraPub():
    def __init__(self, parent: SpotImageServer, namespace: str, compressed: bool = False):
        self.parent = parent
        self.lease_manager = parent.lease_manager

        # RELIABLE for compatibility with the image->pointcloud convertor and
        # other subscribers, depth=1 to stay WiFi-stable
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.info_pub = parent.create_publisher(CameraInfo, '~/' + namespace + '/camera_info', qos_profile=qos_profile)
        if compressed:
            self.image_pub = parent.create_publisher(CompressedImage, '~/' + namespace + '/image/compressed', qos_profile=qos_profile)
        else:
            self.image_pub = parent.create_publisher(Image, '~/' + namespace + '/image', qos_profile=qos_profile)

    def process_data(self, data: ImageResponseProto):
        # Publish if either the image or camera info has subscribers (necessary for nodes like Apriltag).
        has_subscribers = (self.image_pub.get_subscription_count() > 0 or
                           self.info_pub.get_subscription_count() > 0)

        if has_subscribers:
            try:
                image_msg, camera_info_msg, _ = getImageMsg(data, self.lease_manager)
            except UnsupportedImageFormatError as e:
                self.parent.get_logger().warn(f'Skipping frame: {e}', throttle_duration_sec=5.0)
                return
            self.image_pub.publish(image_msg)
            self.info_pub.publish(camera_info_msg)

class SpotImageServer(Node):
    def __init__(self):
        super().__init__('spot_image_server')
        self.get_logger().info('Starting Spot Image Server')

        parameter_listener = spot_driver_parameters.ParamListener(self)
        self.params = parameter_listener.get_params()

        self.get_logger().info(f'Creating image services for the following sources: {", ".join(self.params.image_sources)}')

        self.get_image_service = self.create_service(GetImages, '~/get_images', self.get_image_callback)
        self.list_source_service = self.create_service(Trigger, '~/list_registered_sources', self.list_sources_callback)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Connect to robot
        self.lease_manager = SpotLeaseManager()
        self.lease_manager.setLogger(self.get_logger())
        if not self.lease_manager.connect(self.params.hostname):
            raise RuntimeError('Aborting spot_image_server bringup')

        try:
            self.image_client: ImageClient = self.lease_manager.robot.ensure_client(ImageClient.default_service_name)
        except Exception as e:
            raise RuntimeError(f'Unable to create image client: {e}')

        # Initialize Boston Dynamics image services.
        # The request map is split by consumer so the periodic-publish
        # transport (e.g. JPEG) cannot affect the GetImages service or the
        # static-TF fetch, which always use FORMAT_RAW.
        self.publish_requests: dict[str, ImageRequestProto] = {}
        self.service_requests: dict[str, ImageRequestProto] = {}

        # Bookkeeping of image tasks on the ROS side
        self.camera_pubs: dict[str, CameraPub] = {}
        self.callback_groups: list[MutuallyExclusiveCallbackGroup] = []
        self.publish_timers: list[Timer] = []
        self.image_response_futures: dict[str, Future] = {}

        # Optional per-source frame-rate/latency/bandwidth logging
        self.fps_debug = self.params.fps_debug
        if self.fps_debug:
            self.fps_lock = threading.Lock()
            self.fps_send_times: dict[str, float] = {}
            self.fps_stats: dict[str, dict] = defaultdict(new_fps_bucket)
            self.fps_window_start = time.monotonic()
            self.fps_window_sec = self.params.fps_debug_window
            self.get_logger().info(f'FPS instrumentation enabled, logging a summary every {self.fps_window_sec:.1f}s')

        # Optional compressed RGB transport. Requesting FORMAT_JPEG makes the
        # robot encode onboard, greatly reducing robot->driver WiFi bandwidth.
        self.rgb_jpeg = self.params.rgb_jpeg
        self.jpeg_quality = self.params.jpeg_quality
        if self.rgb_jpeg:
            self.get_logger().warn(
                f'RGB JPEG transport enabled (quality={self.jpeg_quality}). '
                'RGB sources publish CompressedImage on ~/<ns>/image/compressed and the raw '
                '~/<ns>/image topic is not published. Consumers that need raw RGB (e.g. '
                'spot_apriltag, camera_pointclouds) should run: ros2 run image_transport '
                'republish compressed raw --ros-args -r in/compressed:=<ns>/image/compressed '
                '-r out:=<ns>/image. Depth, the get_images service, and static TF remain raw.')

        self.get_logger().info('Creating publishers:')
        for image_source in self.params.image_sources:
            
            if image_source.startswith('hand') and not self.lease_manager.robot.has_arm():
                self.get_logger().warn(f'Robot does not have arm. Skipping image source {image_source}')
                continue

            rgb_source, depth_source = self.resolve_source_name(image_source)

            rgb_pixel_format = image_pb2.Image.PIXEL_FORMAT_RGB_U8 if image_source != 'hand_tof' else None

            # The GetImages service and static-TF fetch always use RAW
            self.service_requests[rgb_source] = build_image_request(rgb_source, image_format=image_pb2.Image.FORMAT_RAW, pixel_format=rgb_pixel_format)
            self.service_requests[depth_source] = build_image_request(depth_source, image_format=image_pb2.Image.FORMAT_RAW)

            # Periodic-publish requests: RGB uses JPEG when enabled (hand_tof is ToF, not RGB); depth is always RAW
            jpeg_for_this = self.rgb_jpeg and image_source != 'hand_tof'
            if jpeg_for_this:
                self.publish_requests[rgb_source] = build_image_request(
                    rgb_source, image_format=image_pb2.Image.FORMAT_JPEG,
                    quality_percent=self.jpeg_quality, pixel_format=rgb_pixel_format)
            else:
                self.publish_requests[rgb_source] = build_image_request(rgb_source, image_format=image_pb2.Image.FORMAT_RAW, pixel_format=rgb_pixel_format)
            self.publish_requests[depth_source] = build_image_request(depth_source, image_format=image_pb2.Image.FORMAT_RAW)

            rgb_rate = self.params.rates.get_entry(image_source).rgb
            depth_rate = self.params.rates.get_entry(image_source).depth

            if rgb_rate > 0:
                self.camera_pubs[rgb_source] = CameraPub(self, 'rgb/' + image_source, compressed=jpeg_for_this)
                self.callback_groups.append(MutuallyExclusiveCallbackGroup())
                self.publish_timers.append(
                    self.create_timer(1/rgb_rate, lambda source=rgb_source: self.update_image_task(source), callback_group=self.callback_groups[-1])
                )
                self.get_logger().info(f'Publishing to rgb/{image_source} at {rgb_rate} Hz')

            if depth_rate > 0:
                self.camera_pubs[depth_source] = CameraPub(self, 'depth/' + image_source)
                self.callback_groups.append(MutuallyExclusiveCallbackGroup())
                self.publish_timers.append(
                    self.create_timer(1/depth_rate, lambda source=depth_source: self.update_image_task(source), callback_group=self.callback_groups[-1])
                )
                self.get_logger().info(f'Publishing to depth/{image_source} at {depth_rate} Hz')

        # Broadcast camera transforms just once post initialization
        self.static_tf_timer = self.create_timer(0.5, self.broadcast_camera_transforms)

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
    
    def update_image_task(self, source_name: str) -> None:
        if source_name not in self.image_response_futures or self.image_response_futures[source_name].done():
            # Do not make requests on images topics that no one is listening to
            cam_pub = self.camera_pubs[source_name]
            is_active_topic = cam_pub.image_pub.get_subscription_count() > 0 or cam_pub.info_pub.get_subscription_count() > 0
            if not is_active_topic: return

            # Record the send time before the callback is registered so a
            # fast-resolving future cannot consume the timestamp first
            if self.fps_debug:
                with self.fps_lock:
                    self.fps_send_times[source_name] = time.monotonic()
            future = self.image_client.get_image_async([self.publish_requests[source_name]])
            self.image_response_futures[source_name] = future
            future.add_done_callback(self.publish_image_callback)

    def publish_image_callback(self, response_future: Future):
        try:
            response: ImageResponseProto = response_future.result()[0]
            if self.fps_debug:
                self.record_fps(response)
            self.camera_pubs[response.source.name].process_data(response)
        except InvalidStateError as e:
            # This path is taken if the image proto has not been returned yet
            # We do nothing and continue waiting for it to arrive
            pass
        except IndexError as e:
            self.get_logger().warn(f'Image future returned with no images to process: {e}')
        except Exception as e:
            self.get_logger().warn(f'Unknown error in image callback: {e}')

    def record_fps(self, response: ImageResponseProto) -> None:
        """Accumulates per-source round-trip latency, frame count, and byte
        volume; emits one summary per window. Logging only."""
        now = time.monotonic()
        source = response.source.name
        nbytes = len(response.shot.image.data)
        with self.fps_lock:
            send_t = self.fps_send_times.pop(source, None)
            st = self.fps_stats[source]
            st['count'] += 1
            st['bytes'] += nbytes
            if send_t is not None:
                lat = now - send_t
                st['lat_sum'] += lat
                st['lat_max'] = max(st['lat_max'], lat)
            elapsed = now - self.fps_window_start
            if elapsed < self.fps_window_sec:
                return
            # Window elapsed: snapshot and reset under the lock
            stats = self.fps_stats
            self.fps_stats = defaultdict(new_fps_bucket)
            self.fps_window_start = now

        # Format + log outside the lock.
        total_bytes = sum(s['bytes'] for s in stats.values())
        lines = [
            f'[fps-debug] {elapsed:.1f}s window | '
            f'{total_bytes / elapsed / 1e6:.1f} MB/s total across '
            f'{len(stats)} active stream(s)'
        ]
        for src in sorted(stats):
            s = stats[src]
            hz = s['count'] / elapsed
            avg_ms = (s['lat_sum'] / s['count'] * 1e3) if s['count'] else 0.0
            max_ms = s['lat_max'] * 1e3
            mbps = s['bytes'] / elapsed / 1e6
            lines.append(
                f'  {src:<32} {hz:5.1f} Hz | lat avg {avg_ms:6.0f} ms '
                f'max {max_ms:6.0f} ms | {mbps:5.1f} MB/s')
        self.get_logger().info('\n'.join(lines))

    def list_sources_callback(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        resp.message = ' '.join([f'[{name}]' for name in self.service_requests.keys()])
        resp.success = True
        return resp

    def get_image_callback(self, req: GetImages.Request, resp: GetImages.Response) -> GetImages.Response:
        resp.success = False

        # Make sure the provided sources were registered on startup
        for source in req.sources:
            if source not in self.service_requests.keys():
                self.get_logger().warn(f'Provided image source {source} does not exist. Registered sources are {self.service_requests.keys()}')
                return resp

        # Request the images from the robot
        try:
            image_responses = self.image_client.get_image([self.service_requests[source] for source in req.sources])
            for response in image_responses:
                if response.status != image_pb2.ImageResponse.Status.STATUS_OK:
                    self.get_logger().warn(f'Unable to retrieve image from {response.source.name}')
                    return resp

                image_msg, camera_info, _ = getImageMsg(response, self.lease_manager)
                resp.images.append(image_msg)
                resp.camera_infos.append(camera_info)

            resp.success = True

        except InvalidRequestError as e:
            self.get_logger().warn(f'There was an error with the request: {e}')
        except RpcError as e:
            self.get_logger().warn(f'Error to communicating with robot: {e}')
        except UnknownImageSourceError as e:
            self.get_logger().warn(f'Provided image source does not exist: {e}')
        except (SourceDataError, UnsetStatusError, ImageDataError) as e:
            self.get_logger().warn(f'Unable to retrive image from robot: {e}')
        except UnsupportedImageFormatError as e:
            self.get_logger().warn(f'Unsupported image format from robot: {e}')

        return resp

    def broadcast_camera_transforms(self):
        """
        Broadcasts static camera transforms for all configured image sources.
        This runs once after node startup to ensure TF listeners receive them.
        Avoids broadcasting duplicate (parent -> child) transforms.
        """
        if hasattr(self, 'static_tf_timer'):
            self.static_tf_timer.cancel()  # Make it oneshot

        if not self.service_requests:
            self.get_logger().info('No image requests to broadcast transforms')
            return

        transform_map = {}  # key: (parent_frame_id, child_frame_id) -> value: TransformStamped

        for response in self.image_client.get_image(list(self.service_requests.values())):
            image_data = response  # ImageResponseProto

            all_tfs_from_data = image_data.shot.transforms_snapshot.child_to_parent_edge_map

            excluded_child_frames = {'odom', 'vision', 'arm0.link_wr1'}

            for child_frame, parent_edge in all_tfs_from_data.items():
                if child_frame in excluded_child_frames:
                    continue
                if not parent_edge.parent_frame_name:
                    continue  # skip invalid entries

                pair_key = (parent_edge.parent_frame_name, child_frame)
                if pair_key in transform_map:
                    continue  # already added, skip

                local_time = self.lease_manager.robotToLocalTime(image_data.shot.acquisition_time)
                tf_time = Time(seconds=local_time.seconds, nanoseconds=local_time.nanos)

                static_tf = populateTransformStamped(
                    tf_time,
                    parent_edge.parent_frame_name,
                    child_frame,
                    parent_edge.parent_tform_child
                )
                transform_map[pair_key] = static_tf  # store it keyed by (parent, child)

        unique_transforms = list(transform_map.values())

        if unique_transforms:
            self.static_tf_broadcaster.sendTransform(unique_transforms)
        else:
            self.get_logger().warn('No static camera transforms found to broadcast')

    def log_transforms(self, tf_transforms):
        """
        Logs a list of TransformStamped messages using the ROS2 logger.
        Args:
            tf_transforms (list of geometry_msgs.msg.TransformStamped): List of transforms.
        """
        for transform in tf_transforms:
            header = transform.header
            trans = transform.transform.translation
            rot = transform.transform.rotation

            msg = (
                "------------------------------\n"
                f"Frame: {header.frame_id} -> {transform.child_frame_id}\n"
                f"  Translation: x={trans.x:.3f}, y={trans.y:.3f}, z={trans.z:.3f}\n"
                f"  Rotation (quaternion): x={rot.x:.3f}, y={rot.y:.3f}, z={rot.z:.3f}, w={rot.w:.3f}"
            )
            self.get_logger().info(msg)

def main():
    rclpy.init()
    try:
        image_server = SpotImageServer()
    except Exception as e:
        rclpy.logging.get_logger('spot_image_server').error(f'{e}')
        exit(1)
    mt_exec = MultiThreadedExecutor(num_threads=4)
    mt_exec.add_node(image_server)
    mt_exec.spin()

    rclpy.shutdown()
