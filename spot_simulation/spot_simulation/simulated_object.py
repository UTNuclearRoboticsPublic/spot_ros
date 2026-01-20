import open3d
import numpy as np
from scipy.spatial.transform import Rotation

class SimulatedObject:
    def __init__(self, object_config):
        self.pose = np.eye(4)
        self.pose[0:3, 3] = np.array(object_config.location)
        self.pose[0:3, 0:3] = Rotation.from_euler(
            seq='ZYX', 
            angles=[object_config.yaw, object_config.pitch, object_config.roll],
            degrees=True
        ).as_matrix()
        
        self.config = object_config

        if self.config.object_type == 'box':
            assert len(self.config.dimensions) == 3, 'Box object type must have 3 dimensions'
            self.geometry = open3d.t.geometry.TriangleMesh.create_box(
                self.config.dimensions[0],
                self.config.dimensions[1],
                self.config.dimensions[2]
            )
            # Box origin by default is its front, bottom, left corner
            self.geometry = self.geometry.translate(-np.array(self.config.dimensions)*0.5)

        elif self.config.object_type == 'cylinder':
            assert len(self.config.dimensions) == 2, 'Cylinder object type must have 2 dimensions'
            self.geometry = open3d.t.geometry.TriangleMesh.create_cylinder(
                height=self.config.dimensions[0],
                radius=self.config.dimensions[1]
            )

        elif self.config.object_type == 'mesh':
            assert len(self.config.file_path), 'No mesh file passed for mesh object'
            self.geometry = open3d.t.geometry.TriangleMesh.from_legacy(
                open3d.io.read_triangle_mesh(self.config.file_path)
            )

        self.geometry = self.geometry.transform(self.pose)