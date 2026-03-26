import open3d
import numpy as np
from sensor_msgs.msg import CameraInfo

class SimulatedDepthCamera:
    def __init__(self, sensor_config):
        self.sensor_config = sensor_config
        self.depth_config = sensor_config.depth_config
        self.pose = open3d.core.Tensor(np.eye(4), dtype=open3d.core.Dtype.Float32)

        self.intrinsic_matrix = open3d.core.Tensor(
            np.array([[self.depth_config.fx, 0.0, self.depth_config.cx],
                      [0.0, self.depth_config.fy, self.depth_config.cy],
                      [0.0,        0.0     ,        1.0     ]]))
        
        self.camera_info = CameraInfo()
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.k = self.intrinsic_matrix.numpy().astype(float).flatten().tolist()
        self.camera_info.p = np.hstack((self.intrinsic_matrix.numpy(), np.zeros((3, 1), dtype=float))).astype(float).flatten().tolist()
        self.camera_info.r = np.eye(3).flatten().tolist()
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.height = self.depth_config.vertical_resolution
        self.camera_info.width = self.depth_config.horizontal_resolution
        self.camera_info.header.frame_id = sensor_config.frame_id

    def generate_rays(self, scene):
        extrinsic = open3d.core.Tensor(np.eye(4), dtype=open3d.core.Dtype.Float32)
        extrinsic[:3, :3] = self.pose[:3, :3].T()
        extrinsic[:3,  3] = (-extrinsic[:3, :3] @ self.pose[:3, 3]).reshape((3,))
        
        rays = scene.create_rays_pinhole(
            self.intrinsic_matrix,
            extrinsic,
            self.depth_config.horizontal_resolution,
            self.depth_config.vertical_resolution
        )
        return rays
