import open3d
import numpy as np
from math import pi
from rclpy.logging import get_logger

class SimulatedDepthCamera:
    def __init__(self, depth_config):
        self.config = depth_config
        self.pose = open3d.core.Tensor(np.eye(4), dtype=open3d.core.Dtype.Float32)

        self.intrinsic_matrix = open3d.core.Tensor(
            np.array([[depth_config.fx, 0.0, depth_config.cx],
                      [0.0, depth_config.fy, depth_config.cy],
                      [0.0,        0.0     ,        1.0     ]]))
        
        # TODO: Store CameraInfo for easy access

    def generate_rays(self, scene):
        return scene.create_rays_pinhole(
            self.intrinsic_matrix,
            self.pose,
            self.config.horizontal_resolution,
            self.config.vertical_resolution
        )
