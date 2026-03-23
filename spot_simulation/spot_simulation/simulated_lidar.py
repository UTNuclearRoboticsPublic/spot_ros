import open3d
import numpy as np

class SimulatedLiDAR:
    def __init__(self, sensor_config):
        self.sensor_config = sensor_config
        self.lidar_config = sensor_config.lidar_config
        self.pose = open3d.core.Tensor(np.eye(4), dtype=open3d.core.Dtype.Float32)

        try:
            self.device = open3d.core.Device(open3d.core.device.SYCL, 0)
        except:
            self.device = open3d.core.Device(open3d.core.Device.CPU, 0)

        self.horizontal_angles = np.linspace(self.lidar_config.min_horizontal_angle, self.lidar_config.max_horizontal_angle, self.lidar_config.num_horizontal_channels)
        self.vertical_angles = np.linspace(self.lidar_config.min_vertical_angle, self.lidar_config.max_vertical_angle, self.lidar_config.num_vertical_channels)
        horizontal_grid, vertical_grid = np.meshgrid(np.deg2rad(self.horizontal_angles), np.deg2rad(self.vertical_angles), indexing='ij')

        sin_horizontal_angles = np.sin(horizontal_grid)
        cos_horizontal_angles = np.cos(horizontal_grid)
        sin_vertical_angles = np.sin(vertical_grid)
        cos_vertical_angles = np.cos(vertical_grid)
        
        ray_directions = np.stack([
            cos_vertical_angles * cos_horizontal_angles,
            cos_vertical_angles * sin_horizontal_angles,
            sin_vertical_angles
        ], axis=-1)
        ray_origins = np.zeros_like(ray_directions)

        rays_numpy = np.stack([ray_origins, ray_directions], axis=1)
        self.ray_definitions = open3d.core.Tensor(rays_numpy.reshape(-1, 6), dtype=open3d.core.Dtype.Float32)

    def generate_rays(self, _):
        rays = open3d.core.Tensor.zeros(shape=list(self.ray_definitions.shape), dtype=open3d.core.Dtype.Float32, device=self.device)

        # Set origin to current sensor position
        rays[:, 0:3] = self.pose[0:3, 3].T()

        # Set the ray directions based on the sensor orientation
        rays[:, 3:] = (self.pose[0:3, 0:3] @ self.ray_definitions[:, 3:].T()).T()

        return rays