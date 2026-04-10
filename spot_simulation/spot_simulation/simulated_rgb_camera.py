import cv2
import rclpy
import open3d
import cv_bridge
import multiprocessing
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from .simulated_object import SimulatedObject

from rclpy.logging import get_logger

class SimulatedRGBCamera:
    render_thread: multiprocessing.Process = None
    max_width = 0
    max_height = 0
    bridge = cv_bridge.CvBridge()
    render_requests = multiprocessing.Queue()
    render_completion_events = {}
    render_buffers = {}
    started = False

    def __init__(self, sensor_config):
        if SimulatedRGBCamera.started and ((sensor_config.rgb_config.vertical_resolution > SimulatedRGBCamera.max_height) or (sensor_config.rgb_config.horizontal_resolution > SimulatedRGBCamera.max_width)):
            raise RuntimeError('Cannot adjust the rendering resolution after the render thread has been started! Add all cameras before starting the thread')

        # Register this camera in the global list
        SimulatedRGBCamera.render_buffers[id(self)] = None
        
        # Record camera config
        self.sensor_config = sensor_config
        self.rgb_config = sensor_config.rgb_config
        self.pose = np.eye(4, dtype=np.float64)
        self.intrinsic_matrix = np.array(
            [[self.rgb_config.fx, 0.0, self.rgb_config.cx],
             [0.0, self.rgb_config.fy, self.rgb_config.cy],
             [0.0,        0.0        ,        1.0        ]],
        dtype=np.float64)
        
        # Set up CameraInfo message. It will be the same for all images except for the timestamp
        self.camera_info = CameraInfo()
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.k = self.intrinsic_matrix.astype(float).flatten().tolist()
        self.camera_info.p = np.hstack((self.intrinsic_matrix, np.zeros((3, 1), dtype=float))).astype(float).flatten().tolist()
        self.camera_info.r = np.eye(3).flatten().tolist()
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.height = self.rgb_config.vertical_resolution
        self.camera_info.width = self.rgb_config.horizontal_resolution
        self.camera_info.header.frame_id = sensor_config.frame_id

        # Keep track of max resolution. Render thread runs at max resolution and resizes down any images that are smaller
        SimulatedRGBCamera.max_height = max(SimulatedRGBCamera.max_height, self.rgb_config.vertical_resolution)
        SimulatedRGBCamera.max_width = max(SimulatedRGBCamera.max_width, self.rgb_config.horizontal_resolution)

    def start(geometries: dict[str, SimulatedObject]):
        # Unpack geometry data into pickle-able data structures
        names = list(geometries.keys())
        colors = [np.array([object.marker.color.r, object.marker.color.g, object.marker.color.b]) for object in geometries.values()]
        vertices = [np.asarray(object.rendering_geometry.vertices) for object in geometries.values()]
        triangles = [np.asarray(object.rendering_geometry.triangles) for object in geometries.values()]

        # Register buffers and multiprocessing events for each object
        for key in SimulatedRGBCamera.render_buffers.keys():
            SimulatedRGBCamera.render_buffers[key] = multiprocessing.Array('B', SimulatedRGBCamera.max_width * SimulatedRGBCamera.max_height * 3)
            SimulatedRGBCamera.render_completion_events[key] = multiprocessing.Event()

        # Start the rendering thread in a new process. The offscreen renderer only works if called by the main thread
        # TODO: Evaluate memory vs. runtime cost of creating a new process for each camera
        SimulatedRGBCamera.render_thread = multiprocessing.Process(
            target=SimulatedRGBCamera.renderingThread,
            args=(names, colors, vertices, triangles)
        )
        SimulatedRGBCamera.render_thread.start()
        SimulatedRGBCamera.started = True

    def renderingThread(names: list[str], colors: list[list[float]], vertices: list[np.ndarray], triangles: list[np.ndarray]):
        get_logger('SimulatedRGBCamera').info(f'Creating an OffscreenRenderer with image size {SimulatedRGBCamera.max_width}x{SimulatedRGBCamera.max_height}')
        render_scene = open3d.visualization.rendering.OffscreenRenderer(SimulatedRGBCamera.max_width, SimulatedRGBCamera.max_height)

        # Reconstruct the geometry and add it to the render scene
        for name, color, vertex_set, triangle_set in zip(names, colors, vertices, triangles):
            geometry = open3d.geometry.TriangleMesh(
                vertices=open3d.utility.Vector3dVector(vertex_set),
                triangles=open3d.utility.Vector3iVector(triangle_set)
            )
            geometry.compute_vertex_normals()
            material = open3d.visualization.rendering.MaterialRecord()
            material.base_color = [*color, 1.0]
            material.shader = "defaultLit"
            render_scene.scene.add_geometry(name, geometry, material)

        while rclpy.ok():
            # Wait for a new request
            req = SimulatedRGBCamera.render_requests.get()
            intrinsic_matrix, extrinsic_matrix, image_width_px, image_height_px, obj_id = req
            rendered_image = np.frombuffer(SimulatedRGBCamera.render_buffers[obj_id].get_obj(), dtype=np.uint8).reshape(SimulatedRGBCamera.max_height, SimulatedRGBCamera.max_width, 3)

            # Render the image
            render_scene.setup_camera(intrinsic_matrix, extrinsic_matrix, image_width_px, image_height_px)
            open3d_image = render_scene.render_to_image()

            # Place into the response queue
            np.copyto(rendered_image, np.asarray(open3d_image))
            SimulatedRGBCamera.render_completion_events[obj_id].set()

    def getImage(self) -> Image:
        if not SimulatedRGBCamera.started:
            raise RuntimeError('Cannot generate image: rendering thread has not been started. Call SimulatedRGBCamera.start!')

        # Submit a render request and wait for it to complete
        extrinsic = np.linalg.inv(self.pose)
        SimulatedRGBCamera.render_requests.put((
            self.intrinsic_matrix,
            extrinsic,
            self.rgb_config.horizontal_resolution,
            self.rgb_config.vertical_resolution,
            id(self)
        ))
        SimulatedRGBCamera.render_completion_events[id(self)].wait()
        SimulatedRGBCamera.render_completion_events[id(self)].clear()

        # After completion, copy to a numpy buffer and set to the image shape
        open3d_image = np.copy(SimulatedRGBCamera.render_buffers[id(self)].get_obj()).reshape(SimulatedRGBCamera.max_height, SimulatedRGBCamera.max_width, 3)

        # Resize and convert to ROS
        cv2_img = cv2.resize(np.asarray(open3d_image), (self.rgb_config.horizontal_resolution, self.rgb_config.vertical_resolution))
        if self.rgb_config.encoding == 'bgr8':
            cv2.cvtColor(cv2_img, cv2.COLOR_RGB2BGR)
        
        return SimulatedRGBCamera.bridge.cv2_to_imgmsg(cv2_img, encoding=self.rgb_config.encoding, header=self.camera_info.header)
