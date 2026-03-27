import os
import xacro
import contextlib
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist, TransformStamped
from urdf_parser_py.urdf import Robot as URDFRobot
from ament_index_python import get_package_share_directory

# Utility function to handle package URI format
def resolve_package(line: str):
    pkg_idx = line.find('package://')
    if pkg_idx == -1:
        start_idx = line.find('file://')
        if start_idx == -1:
            return line
        else:
            return line.removeprefix('file://')

    end_idx = line.find('/', pkg_idx+len('package://')+1)
    package_name = line[pkg_idx + len('package://'):end_idx]
    return line.replace('package://'+package_name, get_package_share_directory(package_name))

class SimulatedRobot:
    def __init__(self, robot_config, node: Node):
        self.node = node
        self.vel = np.array([[0, 0, 0], [0, 0, 0]], dtype=float)
        self.pose = np.eye(4)

        self.pose[:3,  3] = np.array(robot_config.initial_transform.translation)
        self.pose[:3, :3] = Rotation.from_euler(
            seq="ZYX",
            angles=[robot_config.initial_transform.yaw, robot_config.initial_transform.pitch, robot_config.initial_transform.roll],
            degrees=True
        ).as_matrix()

        self.initial_pose = self.pose.copy()
        self.initial_vel = self.vel.copy()

        self.cmd_vel_sub = node.create_subscription(
            msg_type=Twist,
            topic=f'{robot_config.namespace}/cmd_vel',
            callback=self.twist_callback,
            qos_profile=10
        )

        self.odom_pub = node.create_publisher(
            msg_type=Odometry,
            topic=f'{robot_config.namespace}/{robot_config.odometry_topic}',
            qos_profile=10
        )

        xml_string = self.prepare_urdf_file(robot_config.urdf_path, robot_config.xacro_args)
        with contextlib.redirect_stderr(open(os.devnull, 'w')):
            self.urdf_model: URDFRobot = URDFRobot.from_xml_string(xml_string)

        self.transform_pub = TransformBroadcaster(node)
        self.transform = TransformStamped()
        self.transform.header.frame_id = robot_config.odometry_frame
        self.transform.child_frame_id = self.urdf_model.get_root()

    def twist_callback(self, twist: Twist):
        self.vel[0, 0] = twist.linear.x
        self.vel[0, 1] = twist.linear.y
        self.vel[0, 2] = twist.linear.z
        self.vel[1, 0] = twist.angular.x
        self.vel[1, 1] = twist.angular.y
        self.vel[1, 2] = twist.angular.z

    def update_state(self, dt):
        # Update postion using Euler integration
        self.pose[0:3, 3] += self.pose[0:3, 0:3] @ (self.vel[0, :] * dt)

        # Check to see if we're rotation
        speed = np.linalg.norm(self.vel[1, :])
        if speed < 1e-7:
            return
        
        # Update orientation using Rodrigues' formula
        theta = speed * dt
        v = self.vel[1, :] / speed

        skew_sym = np.array([[0, -v[2], v[1]],
                             [v[2], 0, -v[0]],
                             [-v[1], v[0], 0]])
        exp = np.eye(3) + np.sin(theta) * skew_sym + (1 - np.cos(theta)) * (skew_sym @ skew_sym)
        self.pose[:3, :3] = self.pose[:3, :3] @ exp

    def publish_state(self):
        self.transform.transform.translation.x = self.pose[0, 3]
        self.transform.transform.translation.y = self.pose[1, 3]
        self.transform.transform.translation.z = self.pose[2, 3]
        rot = Rotation.from_matrix(self.pose[:3, :3]).as_quat(scalar_first=True)
        self.transform.transform.rotation.w = rot[0]
        self.transform.transform.rotation.x = rot[1]
        self.transform.transform.rotation.y = rot[2]
        self.transform.transform.rotation.z = rot[3]
        self.transform.header.stamp = self.node.get_clock().now().to_msg()

        self.transform_pub.sendTransform(self.transform)

        odom = Odometry()
        odom.header = self.transform.header
        odom.child_frame_id = 'body'
        odom.pose.pose.position.x = self.transform.transform.translation.x
        odom.pose.pose.position.y = self.transform.transform.translation.y
        odom.pose.pose.position.z = self.transform.transform.translation.z
        odom.pose.pose.orientation = self.transform.transform.rotation
        odom.twist.twist.linear.x = float(self.vel[0, 0])
        odom.twist.twist.linear.y = float(self.vel[0, 1])
        odom.twist.twist.linear.z = float(self.vel[0, 2])
        odom.twist.twist.angular.x = float(self.vel[1, 0])
        odom.twist.twist.angular.y = float(self.vel[1, 1])
        odom.twist.twist.angular.z = float(self.vel[1, 2])

        self.odom_pub.publish(odom)

    def prepare_urdf_file(self, filepath: str, xacro_args: str|dict = '') -> str:

        filepath = resolve_package(filepath)
        _, extension = os.path.splitext(filepath)
        self.node.get_logger().info(f'{filepath=}')

        # Get the a list of all lines in the final URDF 
        if extension == '.xacro':
            if type(xacro_args) is str:
                xacro_args = dict(arg.split(":=") for arg in xacro_args.split(" ") if arg)
            urdf_string = xacro.process_file(filepath, mappings=xacro_args).toprettyxml(indent='  ')
            file_strings = urdf_string.split('\n')
        elif extension == '.urdf':
            with open(filepath, 'r') as f:
                file_strings = f.readlines()
        else:
            raise RuntimeError(f'Received robot description file with unsupported format: {extension}')

        for line_idx, line in enumerate(file_strings):
            file_strings[line_idx] = resolve_package(line)

        return '\n'.join(file_strings)
