import rclpy
import open3d
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
from tf2_ros import TransformListener, Buffer, TransformException
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from .simulated_robot import SimulatedRobot
from .simulated_lidar import SimulatedLiDAR
from .simulated_object import SimulatedObject
from .simulation_parameters import simulation_parameters as simulation_parameter_module

SimulationParameters = simulation_parameter_module.Params

class Simulation(Node):
    def __init__(self):
        super().__init__('spot_simulation')

        parameter_listener = simulation_parameter_module.ParamListener(self)
        self.simulation_parameters = parameter_listener.get_params()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
            
        self.objects = {} # List of objects in the scene
        self.robots  = {} # Spot robot
        self.sensors = {} # List of sensors in the scene or on the robot
        self.ids     = {} # All geometry ids in the scene
        self.scene = open3d.t.geometry.RaycastingScene()
        self.sensor_pubs = {}
        self.callback_timers = [] # Timers set to update various things

        self.idx = 0

        for object_name in self.simulation_parameters.object_names:
            self.get_logger().info(f'Loading object "{object_name}": ')
            self.objects[object_name] = SimulatedObject(self.simulation_parameters.objects.get_entry(object_name))
            self.ids[object_name] = self.scene.add_triangles(self.objects[object_name].geometry)
            
        for sensor_name in self.simulation_parameters.sensor_names:
            self.get_logger().info(f'Loading sensor "{sensor_name}"')
            sensor_config = self.simulation_parameters.sensors.get_entry(sensor_name)
            if sensor_config.sensor_type == 'lidar':
                self.sensors[sensor_name] = SimulatedLiDAR(sensor_config.lidar_config)
                self.sensor_pubs[sensor_name] = self.create_publisher(
                    msg_type=PointCloud2,
                    topic=sensor_config.lidar_config.topic,
                    qos_profile=10)
                self.updateSensorTransform(sensor_name)
                self.callback_timers.append(self.create_timer(1.0/sensor_config.update_rate, lambda name=sensor_name: self.updateSensor(name)))
                
        for robot_name in self.simulation_parameters.robot_names:
            self.get_logger().info(f'Loading robot "{robot_name}"')
            robot_config = self.simulation_parameters.robots.get_entry(robot_name)
            self.robots[robot_name] = SimulatedRobot(robot_config, self)
            self.callback_timers.append(self.create_timer(1.0/robot_config.update_rate, lambda name=robot_name: self.robots[name].publish_state()))

        self.update_dt = 0.01
        self.callback_timers.append(self.create_timer(self.update_dt, self.updateRobotTransforms))

    def updateRobotTransforms(self):
        for robot in self.robots.values():
            robot.update_state(self.update_dt)

    def updateSensorTransform(self, sensor_name: str) -> bool:
        sensor = self.sensors.get(sensor_name)
        frame_id = self.simulation_parameters.sensors.get_entry(sensor_name).frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='odom',
                source_frame=frame_id,
                time=Time(),
                timeout=Duration(seconds=0.1)
            )
        except TransformException as e:
            self.get_logger().warn(f'{e}')
            return False
        q = transform.transform.rotation
        d = transform.transform.translation
        sensor.pose[0:3, 3] = np.array([d.x, d.y, d.z])
        rot = Rotation.from_quat([q.w, q.x, q.y, q.z], scalar_first=True)
        sensor.pose[:3, :3] = rot.as_matrix()

        return True

    def updateSensor(self, sensor_name):
        if not self.updateSensorTransform(sensor_name): return

        sensor = self.sensors.get(sensor_name)
        rays = sensor.generate_rays()
        hits = self.scene.cast_rays(rays)
        dists = hits['t_hit']
        dists += np.random.normal(loc=0.0, scale=sensor.config.noise_std_dev, size=dists.shape).astype(np.float32)
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

        header = Header(frame_id=self.simulation_parameters.sensors.get_entry(sensor_name).frame_id, stamp=self.get_clock().now().to_msg())
        pointcloud = create_cloud_xyz32(header, local_points.numpy())
        self.sensor_pubs[sensor_name].publish(pointcloud)


def main():
    rclpy.init()
    simulation = Simulation()
    rclpy.spin(simulation)

if __name__ == '__main__':
    main()