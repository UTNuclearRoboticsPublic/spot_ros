import open3d
import numpy as np
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation
from .simulated_robot import resolve_package

class SimulatedObject:
    object_id = 0

    def __init__(self, object_config, object_name: str):
        self.pose = np.eye(4)
        self.pose[0:3, 3] = np.array(object_config.location)
        self.pose[0:3, 0:3] = Rotation.from_euler(
            seq='ZYX', 
            angles=[object_config.yaw, object_config.pitch, object_config.roll],
            degrees=True
        ).as_matrix()

        self.marker = Marker()
        self.marker.header.frame_id = 'odom'
        self.marker.action = Marker.ADD
        self.marker.id = SimulatedObject.object_id
        self.marker.ns = object_name
        self.marker.pose.position.x = object_config.location[0]
        self.marker.pose.position.y = object_config.location[1]
        self.marker.pose.position.z = object_config.location[2]
        self.marker.color.a = 1.0
        self.marker.color.r = object_config.rgb[0]
        self.marker.color.b = object_config.rgb[1]
        self.marker.color.g = object_config.rgb[2]
        q = Rotation.from_matrix(self.pose[:3, :3]).as_quat(scalar_first=True)
        self.marker.pose.orientation.w = q[0]
        self.marker.pose.orientation.x = q[1]
        self.marker.pose.orientation.y = q[2]
        self.marker.pose.orientation.z = q[3]
        SimulatedObject.object_id += 1
        
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
            self.marker.type = Marker.CUBE
            self.marker.scale.x = self.config.dimensions[0]
            self.marker.scale.y = self.config.dimensions[1]
            self.marker.scale.z = self.config.dimensions[2]

        elif self.config.object_type == 'cylinder':
            assert len(self.config.dimensions) == 2, 'Cylinder object type must have 2 dimensions'
            self.geometry = open3d.t.geometry.TriangleMesh.create_cylinder(
                height=self.config.dimensions[0],
                radius=self.config.dimensions[1]
            )
            self.marker.type = Marker.CYLINDER
            self.marker.scale.x = 2*self.config.dimensions[1]
            self.marker.scale.y = 2*self.config.dimensions[1]
            self.marker.scale.z = self.config.dimensions[0]

        elif self.config.object_type == 'sphere':
            assert len(self.config.dimensions) == 1, 'Sphere object type must have only 1 dimension'
            self.geometry = open3d.t.geometry.TriangleMesh.create_sphere(
                radius=self.config.dimensions[0]
            )
            self.marker.type = Marker.SPHERE
            self.marker.scale.x = 2*self.config.dimensions[0]
            self.marker.scale.y = 2*self.config.dimensions[0]
            self.marker.scale.z = 2*self.config.dimensions[0]

        elif self.config.object_type == 'mesh':
            assert len(self.config.file_path), 'No mesh file passed for mesh object'
            absolute_path = resolve_package(self.config.file_path)
            self.geometry = open3d.t.geometry.TriangleMesh.from_legacy(
                open3d.io.read_triangle_mesh(absolute_path).scale(object_config.mesh_scale, center=np.array([0.0, 0.0, 0.0]))
            )
            self.marker.type = Marker.MESH_RESOURCE
            self.marker.mesh_resource = f'file://{absolute_path}'
            self.marker.scale.x = object_config.mesh_scale
            self.marker.scale.y = object_config.mesh_scale
            self.marker.scale.z = object_config.mesh_scale

        self.geometry = self.geometry.transform(self.pose)
        self.rendering_geometry = self.geometry.to_legacy().paint_uniform_color(object_config.rgb)
