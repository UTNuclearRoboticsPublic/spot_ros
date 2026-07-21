import sys
import rclpy
import open3d
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.publisher import Publisher
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from spot_msgs.srv import SetSimulatedPose
from visualization_msgs.msg import MarkerArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from scipy.spatial.transform import Rotation
from tf2_ros import TransformListener, Buffer, TransformException
from tf2_geometry_msgs import PoseStamped # needed tf_buffer.transform
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from .simulated_robot import SimulatedRobot
from .simulated_lidar import SimulatedLiDAR
from .simulated_object import SimulatedObject
from .simulated_rgb_camera import SimulatedRGBCamera
from .simulated_depth_camera import SimulatedDepthCamera
from .simulation_parameters import simulation_parameters as simulation_parameter_module

SimulationParameters = simulation_parameter_module.Params

class Simulation(Node):
    def __init__(self):
        super().__init__('spot_simulation')

        parameter_listener = simulation_parameter_module.ParamListener(self)
        self.simulation_parameters = parameter_listener.get_params()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
            
        self.objects: dict[str, SimulatedObject] = {} # List of objects in the scene
        self.robots: dict[str, SimulatedRobot]  = {} # Spot robot
        self.sensors: dict[str, SimulatedLiDAR|SimulatedDepthCamera] = {} # List of sensors in the scene or on the robot
        self.scene = open3d.t.geometry.RaycastingScene()
        self.sensor_pubs: dict[str, Publisher] = {}
        self.sensor_info_pubs: dict[str, Publisher] = {} # For cameras
        self.callback_timers = [] # Timers set to update various things

        self.idx = 0

        latching_qos = QoSProfile(depth=1)
        latching_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.geometry_markers = self.create_publisher(MarkerArray, '~/simulation_geometry', latching_qos)

        environment_markers = MarkerArray()
        for object_name in self.simulation_parameters.object_names:
            self.get_logger().info(f'Loading object "{object_name}": ')
            self.objects[object_name] = SimulatedObject(self.simulation_parameters.objects.get_entry(object_name), object_name)
            self.scene.add_triangles(self.objects[object_name].geometry)
            environment_markers.markers.append(self.objects[object_name].marker)
        self.geometry_markers.publish(environment_markers)

        self.sensor_callback_group = ReentrantCallbackGroup()
        for sensor_name in self.simulation_parameters.sensor_names:
            self.get_logger().info(f'Loading sensor "{sensor_name}"')
            sensor_config = self.simulation_parameters.sensors.get_entry(sensor_name)
            if sensor_config.sensor_type == 'lidar':
                self.sensors[sensor_name] = SimulatedLiDAR(sensor_config)
                self.sensor_pubs[sensor_name] = self.create_publisher(
                    msg_type=PointCloud2,
                    topic=sensor_config.topic,
                    qos_profile=10 if not sensor_config.best_effort else qos_profile_sensor_data)
            elif sensor_config.sensor_type == 'depth_camera':
                self.sensors[sensor_name] = SimulatedDepthCamera(sensor_config)
                self.sensor_pubs[sensor_name] = self.create_publisher(
                    msg_type=Image,
                    topic=sensor_config.topic,
                    qos_profile=10 if not sensor_config.best_effort else qos_profile_sensor_data
                )
                self.sensor_info_pubs[sensor_name] = self.create_publisher(
                    msg_type=CameraInfo,
                    topic=sensor_config.depth_config.info_topic,
                    qos_profile=10 if not sensor_config.best_effort else qos_profile_sensor_data
                )
            elif sensor_config.sensor_type == 'rgb_camera':
                self.sensors[sensor_name] = SimulatedRGBCamera(sensor_config)
                self.sensor_pubs[sensor_name] = self.create_publisher(
                    msg_type=Image,
                    topic=sensor_config.topic,
                    qos_profile=10 if not sensor_config.best_effort else qos_profile_sensor_data
                )
                self.sensor_info_pubs[sensor_name] = self.create_publisher(
                    msg_type=CameraInfo,
                    topic=sensor_config.rgb_config.info_topic,
                    qos_profile=10 if not sensor_config.best_effort else qos_profile_sensor_data
                )

            self.callback_timers.append(self.create_timer(1.0/sensor_config.update_rate, lambda name=sensor_name: self.updateSensor(name), self.sensor_callback_group))

        # Start the camera rendering thread
        SimulatedRGBCamera.start(self.objects)

        for robot_name in self.simulation_parameters.robot_names:
            self.get_logger().info(f'Loading robot "{robot_name}"')
            robot_config = self.simulation_parameters.robots.get_entry(robot_name)
            self.robots[robot_name] = SimulatedRobot(robot_config, self)
            self.callback_timers.append(self.create_timer(1.0/robot_config.update_rate, lambda name=robot_name: self.robots[name].publish_state()))

        self.service_callback_group = MutuallyExclusiveCallbackGroup()
        self.reset_server = self.create_service(Trigger, "~/reset_simulation", self.resetRobotTransforms, callback_group=self.service_callback_group)
        self.pose_server = self.create_service(SetSimulatedPose, "~/set_robot_pose", self.setRobotPose, callback_group=self.service_callback_group)

        self.update_dt = 0.01
        self.callback_timers.append(self.create_timer(self.update_dt, self.updateRobotTransforms, MutuallyExclusiveCallbackGroup()))

    def updateRobotTransforms(self) -> None:
        for robot in self.robots.values():
            robot.update_state(self.update_dt)

    def resetRobotTransforms(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        for robot in self.robots.values():
            robot.pose = robot.initial_pose.copy()
            robot.vel = robot.initial_vel.copy()
        resp.success = True
        resp.message = "Simulation Reset"
        return resp

    def setRobotPose(self, req: SetSimulatedPose.Request, resp: SetSimulatedPose.Response) -> SetSimulatedPose.Response:
        if req.robot_name not in self.robots:
            self.get_logger().warn(f'Unable to set the pose of unknown robot "{req.robot_name}"')
            resp.success = False
            return resp
        robot = self.robots[req.robot_name]
        
        try:
            world_pose = self.tf_buffer.transform(req.pose, self.simulation_parameters.world_frame, Duration(seconds=1.0))
        except TransformException as e:
            self.get_logger().warn(f'Unable to set the pose of robot: {e}\nTrying again at the current timestamp')
            try:
                req.pose.header.stamp.sec = req.pose.header.stamp.nanosec = 0
                world_pose = self.tf_buffer.transform(req.pose, self.simulation_parameters.world_frame, Duration(seconds=1.0))
            except TransformException as e:
                self.get_logger().error(f'Still failed: {e}\n')
                resp.success = False
                return resp

        q = world_pose.pose.orientation
        d = world_pose.pose.position
        rot = Rotation.from_quat([q.w, q.x, q.y, q.z], scalar_first=True)
        if req.se2:
            robot.pose[0:2, 3] = np.array([d.x, d.y])
            yaw, _, _ = rot.as_euler('ZYX')
            robot.pose[:3, :3] = Rotation.from_rotvec(np.array([0, 0, yaw])).as_matrix()
        else:
            robot.pose[0:3, 3] = np.array([d.x, d.y, d.z])
            robot.pose[:3, :3] = rot.as_matrix()

        self.get_logger().info(f'Set the robot pose to [x: {d.x:.3f}, y: {d.y:.3f}, z: {d.z:.3f}] | [w: {q.w:.3f}, x: {q.x:.3f}, y: {q.y:.3f}, z: {q.z:.3f}]')

        resp.success = True
        return resp

    def updateSensorTransform(self, sensor_name: str, timestamp: Time) -> bool:
        sensor = self.sensors.get(sensor_name)
        frame_id = self.simulation_parameters.sensors.get_entry(sensor_name).frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.simulation_parameters.world_frame,
                source_frame=frame_id,
                time=timestamp,
                timeout=Duration(seconds=0.1)
            )
        except TransformException as e:
            return False
        q = transform.transform.rotation
        d = transform.transform.translation
        sensor.pose[0:3, 3] = np.array([d.x, d.y, d.z])
        rot = Rotation.from_quat([q.w, q.x, q.y, q.z], scalar_first=True)
        sensor.pose[:3, :3] = rot.as_matrix()

        return True

    def updateSensor(self, sensor_name) -> None:
        timestamp = self.get_clock().now()
        if not self.updateSensorTransform(sensor_name, timestamp): return

        sensor: SimulatedDepthCamera | SimulatedLiDAR | SimulatedRGBCamera = self.sensors.get(sensor_name)

        # Handle the easy case of an RGB camera
        if type(sensor) is SimulatedRGBCamera:
            image: Image = sensor.getImage()
            image.header.stamp = timestamp.to_msg()
            sensor.camera_info.header.stamp = image.header.stamp
            self.sensor_pubs[sensor_name].publish(image)
            self.sensor_info_pubs[sensor_name].publish(sensor.camera_info)
            return

        # Otherwise it's a depth sensor and we have to cast rays
        rays = sensor.generate_rays(self.scene)
        hits = self.scene.cast_rays(rays)
        dists = hits['t_hit']
        dists += np.random.normal(loc=0.0, scale=sensor.sensor_config.noise_std_dev, size=dists.shape).astype(np.float32)

        valid_dists = dists.isfinite() & (dists > sensor.sensor_config.min_range) & (dists < sensor.sensor_config.max_range)

        # Flatten depth images to help with math
        if type(sensor) == SimulatedDepthCamera:
            rays = rays.reshape((-1, 6))
            dists[valid_dists.logical_not()] = 0.0
            dists = dists.flatten()
        else:
            dists[valid_dists.logical_not()] = np.inf
        
        # Handle poor broadcasting ability of Open3D tensors
        rays[:, 3] *= dists
        rays[:, 4] *= dists
        rays[:, 5] *= dists
        points = rays[:, 0:3] + rays[:, 3:] # defined in the simulation frame

        # Transform points back to sensor frame
        world_tform_sensor = sensor.pose
        sensor_tform_world_rot = world_tform_sensor[:3, :3].T()
        sensor_tform_world_trans = -sensor_tform_world_rot @ world_tform_sensor[:3, 3]
        local_points = (sensor_tform_world_rot @ points.T()).T() 
        local_points[:, 0] += sensor_tform_world_trans[0]
        local_points[:, 1] += sensor_tform_world_trans[1]
        local_points[:, 2] += sensor_tform_world_trans[2]

        if type(sensor) is SimulatedLiDAR:
            header = Header(
                frame_id=sensor.sensor_config.frame_id, 
                stamp=self.get_clock().now().to_msg()
            )
            pointcloud = create_cloud_xyz32(header, local_points.numpy())
            self.sensor_pubs[sensor_name].publish(pointcloud)

        elif type(sensor) is SimulatedDepthCamera:
            sensor.camera_info.header.stamp = self.get_clock().now().to_msg()

            # Floating point depth in meters
            depths = local_points[:, 2].numpy()

            depth_image = Image()
            depth_image.header = sensor.camera_info.header
            depth_image.height = sensor.camera_info.height
            depth_image.width = sensor.camera_info.width
            depth_image.encoding = sensor.depth_config.encoding
            np.nan_to_num(depths, nan=0.0, posinf=0.0, neginf=0.0, copy=False)
            if depth_image.encoding == '16UC1':
                depth_image.data = (depths * 1000.0).astype(np.uint16).tobytes()
                depth_image.step = sensor.depth_config.horizontal_resolution * 2
            elif depth_image.encoding == '32FC1':
                depth_image.data = depths.tobytes()
                depth_image.step = sensor.depth_config.horizontal_resolution * 4
            depth_image.is_bigendian = 0 if sys.byteorder == "little" else 1

            self.sensor_pubs[sensor_name].publish(depth_image)
            self.sensor_info_pubs[sensor_name].publish(sensor.camera_info)



def main():
    rclpy.init()
    exec = MultiThreadedExecutor()
    simulation = Simulation()
    exec.add_node(simulation)
    exec.spin()

if __name__ == '__main__':
    main()