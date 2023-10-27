############################################################################################
#      Title     : ros_helpers.py
#      Project   : spot_ros
#      Copyright : Copyright© The University of Texas at Austin, 2022. All rights reserved.
#                
#          All files within this directory are subject to the following, unless an alternative
#          license is explicitly included within the text of each file.
#
#          This software and documentation constitute an unpublished work
#          and contain valuable trade secrets and proprietary information
#          belonging to the University. None of the foregoing material may be
#          copied or duplicated or disclosed without the express, written
#          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
#          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
#          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
#          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
#          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
#          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
#          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
#          University be liable for incidental, special, indirect, direct or
#          consequential damages or loss of profits, interruption of business,
#          or related expenses which may arise from use of software or documentation,
#          including but not limited to those resulting from defects in software
#          and/or documentation, or loss or inaccuracy of data of any kind.
#
############################################################################################

from typing import List, Text, Tuple
import struct
from numpy import float32
from math import nan

import rclpy.time

from .spot_wrapper import SpotWrapper
from scipy import spatial

from builtin_interfaces.msg import Time as ROSTime
from builtin_interfaces.msg import Duration as ROSDuration
from geometry_msgs.msg import PoseWithCovariance, TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from spot_msgs.msg import DockState
from spot_msgs.msg import FootState, FootStateArray
from spot_msgs.msg import EStopState, EStopStateArray
from spot_msgs.msg import WiFiState
from spot_msgs.msg import PowerState
from spot_msgs.msg import BehaviorFault, BehaviorFaultState
from spot_msgs.msg import SystemFault, SystemFaultState
from spot_msgs.msg import BatteryState, BatteryStateArray
from spot_msgs.msg import ManipulatorState

from bosdyn.api import image_pb2, robot_state_pb2, service_fault_pb2
from bosdyn.api.docking import docking_pb2
from bosdyn.client.math_helpers import SE3Pose
from bosdyn.client.frame_helpers import get_odom_tform_body, get_vision_tform_body

"""Dictionaries for mapping BD joint names to more friendly names"""
body_joint_names = {
    'fl.hx' : 'front_left_hip_x',
    'fl.hy' : 'front_left_hip_y',
    'fl.kn' : 'front_left_knee',
    'fr.hx' : 'front_right_hip_x',
    'fr.hy' : 'front_right_hip_y',
    'fr.kn' : 'front_right_knee',
    'hl.hx' : 'rear_left_hip_x',
    'hl.hy' : 'rear_left_hip_y',
    'hl.kn' : 'rear_left_knee',
    'hr.hx' : 'rear_right_hip_x',
    'hr.hy' : 'rear_right_hip_y',
    'hr.kn' : 'rear_right_knee',
}

arm_joint_names = {
    'arm0.sh0' : 'arm0_shoulder_yaw',
    'arm0.sh1' : 'arm0_shoulder_pitch',
    'arm0.hr0' : 'arm0_shoulder_roll',
    'arm0.el0' : 'arm0_elbow_pitch',
    'arm0.el1' : 'arm0_elbow_roll',
    'arm0.wr0' : 'arm0_wrist_pitch',
    'arm0.wr1' : 'arm0_wrist_roll',
    'arm0.f1x' : 'arm0_fingers'
}

friendly_joint_names = dict(body_joint_names, **arm_joint_names)

def populateTransformStamped(time: rclpy.time.Time,
                             parent_frame: Text,
                             child_frame: Text,
                             transform: SE3Pose) -> TransformStamped:
    """Populates a TransformStamped message

    Args:
        time: The time of the transform
        parent_frame: The parent frame of the transform
        child_frame: The child_frame_id of the transform
        transform: A transform to copy into a StampedTransform object. Should have position (x,y,z) and rotation (x,
        y,z,w) members
    Returns:
        TransformStamped message
    """
    new_tf = TransformStamped()
    new_tf.header.stamp = time.to_msg()
    new_tf.header.frame_id = parent_frame
    new_tf.child_frame_id = child_frame
    new_tf.transform.translation.x = transform.position.x
    new_tf.transform.translation.y = transform.position.y
    new_tf.transform.translation.z = transform.position.z
    new_tf.transform.rotation.x = transform.rotation.x
    new_tf.transform.rotation.y = transform.rotation.y
    new_tf.transform.rotation.z = transform.rotation.z
    new_tf.transform.rotation.w = transform.rotation.w

    return new_tf

def getImageMsg(data: image_pb2.ImageResponse, spot_wrapper: SpotWrapper) -> Tuple[Image, CameraInfo, TFMessage]:
    """Takes the image, camera, and TF data and populates the necessary ROS messages

    Args:
        data: ImageResponse proto
        spot_wrapper: A SpotWrapper object
    Returns:
        (tuple):
            * Image: message of the image captured
            * CameraInfo: message to define the state and config of the camera that took the image
            * TFMessage: with the transforms necessary to locate the image frames
    """
    tf_msg = TFMessage()
    for frame_name in data.shot.transforms_snapshot.child_to_parent_edge_map:
        if data.shot.transforms_snapshot.child_to_parent_edge_map.get(frame_name).parent_frame_name:
            transform = data.shot.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
            new_tf = TransformStamped()
            local_time = spot_wrapper.robotToLocalTime(data.shot.acquisition_time)
            new_tf.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
            new_tf.header.frame_id = transform.parent_frame_name
            new_tf.child_frame_id = frame_name
            new_tf.transform.translation.x = transform.parent_tform_child.position.x
            new_tf.transform.translation.y = transform.parent_tform_child.position.y
            new_tf.transform.translation.z = transform.parent_tform_child.position.z
            new_tf.transform.rotation.x = transform.parent_tform_child.rotation.x
            new_tf.transform.rotation.y = transform.parent_tform_child.rotation.y
            new_tf.transform.rotation.z = transform.parent_tform_child.rotation.z
            new_tf.transform.rotation.w = transform.parent_tform_child.rotation.w
            tf_msg.transforms.append(new_tf)

    image_msg = Image()
    local_time = spot_wrapper.robotToLocalTime(data.shot.acquisition_time)
    image_msg.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
    image_msg.header.frame_id = data.shot.frame_name_image_sensor
    image_msg.height = data.shot.image.rows
    image_msg.width = data.shot.image.cols

    # Color/greyscale formats.
    # JPEG format
    if data.shot.image.format == image_pb2.Image.FORMAT_JPEG:
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = True
        image_msg.step = 3 * data.shot.image.cols
        image_msg.data = data.shot.image.data

    # Uncompressed.  Requires pixel_format.
    elif data.shot.image.format == image_pb2.Image.FORMAT_RAW:
        # One byte per pixel.
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
            image_msg.encoding = 'mono8'
            image_msg.is_bigendian = True
            image_msg.step = data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Three bytes per pixel.
        elif data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
            image_msg.encoding = 'rgb8'
            image_msg.is_bigendian = True
            image_msg.step = 3 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Four bytes per pixel.
        elif data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
            image_msg.encoding = 'rgba8'
            image_msg.is_bigendian = True
            image_msg.step = 4 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # UInt16 greyscale
        elif data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U16:
            image_msg.encoding = 'mono16'
            image_msg.is_bigendian = False
            image_msg.step = 2 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Depth image. See ROS encoding convention: https://www.ros.org/reps/rep-0118.html
        # Spot SDK outputs in OpenNI format (Little-endian uint16 z-distance from camera in mm)
        elif data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            image_msg.encoding = '16UC1'
            image_msg.is_bigendian = False
            image_msg.step = 2 * data.shot.image.cols
            image_msg.data = data.shot.image.data

            # image_msg.encoding = '32FC1'
            # image_msg.is_bigendian = False
            # image_msg.step = 2 * data.shot.image.cols

            # # convert the uint16's into 32-bit floats
            # # depth zero in OpenNI format is converted to NaN
            # assert data.shot.image.data, 'No camera data received for 32FC1 encoding.'
            # assert len(data.shot.image.data) % 2 == 0, 'Received odd number of bytes for  32FC1 encoding.'
            # image_msg.data = []

            # # iterate over the list two bytes at a time
            # for i,j in zip(data.shot.image.data[::2], data.shot.image.data[1::2]):
            #     pixel = int(i * 256 + j)
            #     value_in_meters = float32(nan)

            #     if pixel != 0:
            #         value_in_meters = float32(pixel / data.source.depth_scale)
                
            #     bytes = list(struct.pack('<f', value_in_meters))
            #     image_msg.data.extend(bytes)

    elif data.shot.image.format == image_pb2.Image.PIXEL_FORMAT_UNKNOWN:
        spot_wrapper.logger.error('Unknown image format from Spot SDK.', throttle_duration_sec=5.0)
        return Image(), CameraInfo(), tf_msg

    camera_info_msg = CameraInfo(d=[0]*5,
                                 distortion_model="plumb_bob",
                                 k=[0,0,0,0,0,0,0,0,1],
                                 r=[1,0,0,0,1,0,0,0,1],
                                 p=[0,0,0,0,0,0,0,0,0,0,1,0])

    local_time = spot_wrapper.robotToLocalTime(data.shot.acquisition_time)
    camera_info_msg.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
    camera_info_msg.header.frame_id = data.shot.frame_name_image_sensor
    camera_info_msg.height = data.shot.image.rows
    camera_info_msg.width = data.shot.image.cols

    camera_info_msg.k[0] = data.source.pinhole.intrinsics.focal_length.x
    camera_info_msg.k[2] = data.source.pinhole.intrinsics.principal_point.x
    camera_info_msg.k[4] = data.source.pinhole.intrinsics.focal_length.y
    camera_info_msg.k[5] = data.source.pinhole.intrinsics.principal_point.y

    camera_info_msg.p[0] = data.source.pinhole.intrinsics.focal_length.x
    camera_info_msg.p[2] = data.source.pinhole.intrinsics.principal_point.x
    camera_info_msg.p[5] = data.source.pinhole.intrinsics.focal_length.y
    camera_info_msg.p[6] = data.source.pinhole.intrinsics.principal_point.y

    return image_msg, camera_info_msg, tf_msg

def JointStatesToMsg(kinematic_state: robot_state_pb2.KinematicState,
                     spot_wrapper: SpotWrapper) -> JointState:
    """Maps joint state data from robot state proto to ROS JointState message

    Args:
        kinematic_state: KinematicState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        sensor_msgs/JointState ROS message
    """
    # static attributes of this method
    joint_state_msg = JointState()
    local_time = spot_wrapper.robotToLocalTime(kinematic_state.acquisition_timestamp)
    joint_state_msg.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)

    for joint in kinematic_state.joint_states:
        try:
            name = friendly_joint_names[joint.name]
        except KeyError:
            spot_wrapper.logger.error('Failed to look up friendly name for frame ' + joint.name,
                                       once=True)
            continue
        
        joint_state_msg.name.append(name)
        joint_state_msg.position.append(joint.position.value)
        joint_state_msg.velocity.append(joint.velocity.value)
        joint_state_msg.effort.append(joint.load.value)

    return joint_state_msg

def EStopStatesToMsg(estop_states: robot_state_pb2.EStopState,
                     spot_wrapper: SpotWrapper) -> EStopStateArray:
    """Maps EStop states data from robot state proto to ROS EStopArray message

    Args:
        estop_states: EStopState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        spot_msgs/EStopArray ROS message
    """
    estop_array_msg = EStopStateArray()
    for estop in estop_states:
        estop_msg = EStopState()
        local_time = spot_wrapper.robotToLocalTime(estop.timestamp)
        estop_msg.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
        estop_msg.name = estop.name
        estop_msg.type = estop.type
        estop_msg.state = estop.state
        estop_array_msg.estop_states.append(estop_msg)

    return estop_array_msg

def FeetStateToMsg(foot_states: robot_state_pb2.FootState) -> FootStateArray:
    """Maps foot position state data from robot state proto to ROS FootStateArray message

    Args:
        foot_states: FootState proto
    Returns:
        spot_msgs/FootStateArray ROS message
    """
    foot_array_msg = FootStateArray()
    for foot in foot_states:
        foot_msg = FootState()
        foot_msg.foot_position_rt_body.x = foot.foot_position_rt_body.x
        foot_msg.foot_position_rt_body.y = foot.foot_position_rt_body.y
        foot_msg.foot_position_rt_body.z = foot.foot_position_rt_body.z
        foot_msg.contact = foot.contact
        foot_array_msg.states.append(foot_msg)

    return foot_array_msg

def GetOdomTwistFromState(kinematic_state: robot_state_pb2.KinematicState,
                          spot_wrapper: SpotWrapper) -> TwistWithCovarianceStamped:
    """Maps odometry data from robot state proto to ROS TwistWithCovarianceStamped message

    Args:
        kinematic_state: KinematicState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        geometry_msgs/TwistWithCovarianceStamped ROS message
    """
    twist_odom_msg = TwistWithCovarianceStamped()
    local_time = spot_wrapper.robotToLocalTime(kinematic_state.acquisition_timestamp)
    twist_odom_msg.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
    twist_odom_msg.twist.twist.linear.x = kinematic_state.velocity_of_body_in_odom.linear.x
    twist_odom_msg.twist.twist.linear.y = kinematic_state.velocity_of_body_in_odom.linear.y
    twist_odom_msg.twist.twist.linear.z = kinematic_state.velocity_of_body_in_odom.linear.z
    twist_odom_msg.twist.twist.angular.x = kinematic_state.velocity_of_body_in_odom.angular.x
    twist_odom_msg.twist.twist.angular.y = kinematic_state.velocity_of_body_in_odom.angular.y
    twist_odom_msg.twist.twist.angular.z = kinematic_state.velocity_of_body_in_odom.angular.z
    return twist_odom_msg

def GetOdomFromState(kinematic_state: robot_state_pb2.KinematicState,
                     spot_wrapper: SpotWrapper,
                     use_vision: bool) -> Odometry:
    """Maps odometry data from robot state proto to ROS Odometry message

    Args:
        kinematic_state: KinematicState proto
        spot_wrapper: A SpotWrapper object
        use_vision: If true, use visual odometry in addition to kinematic odometry
    Returns:
        nav_msgs/Odometry ROS message
    """
    odom_msg = Odometry()
    local_time = spot_wrapper.robotToLocalTime(kinematic_state.acquisition_timestamp)
    odom_msg.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
    if use_vision == True:
        odom_msg.header.frame_id = 'vision'
        tform_body = get_vision_tform_body(kinematic_state.transforms_snapshot)
    else:
        odom_msg.header.frame_id = 'odom'
        tform_body = get_odom_tform_body(kinematic_state.transforms_snapshot)
    odom_msg.child_frame_id = 'body'
    pose_odom_msg = PoseWithCovariance()
    pose_odom_msg.pose.position.x = tform_body.position.x
    pose_odom_msg.pose.position.y = tform_body.position.y
    pose_odom_msg.pose.position.z = tform_body.position.z
    pose_odom_msg.pose.orientation.x = tform_body.rotation.x
    pose_odom_msg.pose.orientation.y = tform_body.rotation.y
    pose_odom_msg.pose.orientation.z = tform_body.rotation.z
    pose_odom_msg.pose.orientation.w = tform_body.rotation.w

    odom_msg.pose = pose_odom_msg
    twist_odom_msg = GetOdomTwistFromState(kinematic_state, spot_wrapper).twist
    odom_msg.twist = twist_odom_msg
    return odom_msg

def DockStateToMsg(dock_state: docking_pb2.DockState) -> DockState:
    """Maps dock state data from robot state proto to ROS DockState message
    Args:
        dock_state: DockState proto
    Returns:
        spot_msgs/DockState ROS message
    """
    dock_state_msg = DockState()
    dock_state_msg.status = dock_state.status
    dock_state_msg.dock_type = dock_state.dock_type
    dock_state_msg.dock_id = dock_state.dock_id
    dock_state_msg.power_status = dock_state.power_status
    return dock_state_msg


def GetWifiFromState(comms_states: robot_state_pb2.CommsState) -> WiFiState:
    """Maps wireless state data from robot state proto to ROS WiFiState message

    Args:
        data: CommsState proto
    Returns:
        spot_msgs/WiFiState ROS message
    """
    wifi_msg = WiFiState()
    for comm_state in comms_states:
        if comm_state.HasField('wifi_state'):
            wifi_msg.current_mode = comm_state.wifi_state.current_mode
            wifi_msg.essid = comm_state.wifi_state.essid

    return wifi_msg

def invertTransform(transform: TransformStamped) -> TransformStamped:
    """Calculates and return the inverse of a geometry_msgs/TransformStamped
    The new transform will have the same time stamp, but the parent and
    child frame ID's will be swapped and the transformtation inverted
    
    Args:
        transform: TransformStamped
    Returns:
        TransformStamped
    """
    
    # Extract the components of the transformation
    rotation = spatial.transform.Rotation.from_quat([
        transform.transform.rotation.x, 
        transform.transform.rotation.y, 
        transform.transform.rotation.z, 
        transform.transform.rotation.w
    ])
    translation = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]

    # Invert the individual components
    inv_rotation = rotation.inv()
    inv_translation = -1.0*inv_rotation.apply(translation)

    # Create the inverse transform
    inverse = TransformStamped()
    inverse.header.stamp    = transform.header.stamp
    inverse.header.frame_id = transform.child_frame_id
    inverse.child_frame_id  = transform.header.frame_id

    inverse.transform.translation.x = inv_translation[0]
    inverse.transform.translation.y = inv_translation[1]
    inverse.transform.translation.z = inv_translation[2]

    q_inv = inv_rotation.as_quat()
    inverse.transform.rotation.x = q_inv[0]
    inverse.transform.rotation.y = q_inv[1]
    inverse.transform.rotation.z = q_inv[2]
    inverse.transform.rotation.w = q_inv[3]

    return inverse

def createBaseFootprintTransform(transform_odom2body: TransformStamped, transform_odom2gpe: TransformStamped):
    transform_body2basefootprint = TransformStamped()
    transform_body2basefootprint.header.frame_id = "base_link"
    transform_body2basefootprint.child_frame_id  = "base_footprint"
    transform_body2basefootprint.header.stamp = transform_odom2body.header.stamp

    transform_body2basefootprint.transform.translation.x = transform_odom2gpe.transform.translation.x - transform_odom2body.transform.translation.x
    transform_body2basefootprint.transform.translation.y = transform_odom2gpe.transform.translation.y - transform_odom2body.transform.translation.y
    transform_body2basefootprint.transform.translation.z = transform_odom2gpe.transform.translation.z - transform_odom2body.transform.translation.z
    transform_body2basefootprint.transform.rotation.w = 1.0

    return transform_body2basefootprint

def GetTFFromState(kinematic_state: robot_state_pb2.KinematicState,
                   spot_wrapper: SpotWrapper) -> TFMessage:
    """Maps robot link state data from robot state proto to ROS TFMessage message

    Args:
        kinematic_state: KinematicState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        tf2_msgs/TFMessage message
    """
    tf_msg = TFMessage()

    transform_odom2body = None
    transform_odom2gpe = None

    for frame_name in kinematic_state.transforms_snapshot.child_to_parent_edge_map:
        if kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name).parent_frame_name:
            transform = kinematic_state.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
            new_tf = TransformStamped()
            local_time = spot_wrapper.robotToLocalTime(kinematic_state.acquisition_timestamp)
            new_tf.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
            new_tf.header.frame_id = transform.parent_frame_name
            new_tf.child_frame_id = frame_name
            new_tf.transform.translation.x = transform.parent_tform_child.position.x
            new_tf.transform.translation.y = transform.parent_tform_child.position.y
            new_tf.transform.translation.z = transform.parent_tform_child.position.z
            new_tf.transform.rotation.x = transform.parent_tform_child.rotation.x
            new_tf.transform.rotation.y = transform.parent_tform_child.rotation.y
            new_tf.transform.rotation.z = transform.parent_tform_child.rotation.z
            new_tf.transform.rotation.w = transform.parent_tform_child.rotation.w

            # Account for the fact that Spot publishes a body->odom transform but we want odom->body
            if frame_name == "odom":
                new_tf = invertTransform(new_tf)
                transform_odom2body = new_tf

            # Record the odom -> gpe transform for later use
            if frame_name == "gpe":
                transform_odom2gpe = new_tf

            tf_msg.transforms.append(new_tf)

    # Create a base_footprint transform from the gpe transform
    if transform_odom2body is not None and transform_odom2gpe is not None:
        tf_msg.transforms.append(createBaseFootprintTransform(transform_odom2body, transform_odom2gpe))

    return tf_msg

def BatteryStatesToMsg(battery_states: robot_state_pb2.BatteryState,
                       spot_wrapper: SpotWrapper) -> BatteryStateArray:
    """Maps battery state data from robot state proto to ROS BatteryStateArray message

    Args:
        battery_states: BatteryState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        spot_msgs/BatteryStateArray ROS message
    """
    battery_states_array_msg = BatteryStateArray()
    for battery in battery_states:
        battery_msg = BatteryState()
        local_time = spot_wrapper.robotToLocalTime(battery.timestamp)
        battery_msg.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)

        battery_msg.identifier = battery.identifier
        battery_msg.charge_percentage = battery.charge_percentage.value
        battery_msg.estimated_runtime = ROSDuration(sec=battery.estimated_runtime.seconds, nanosec=battery.estimated_runtime.nanos)
        battery_msg.current = battery.current.value
        battery_msg.voltage = battery.voltage.value
        for temp in battery.temperatures:
            battery_msg.temperatures.append(temp)
        battery_msg.status = battery.status
        battery_states_array_msg.battery_states.append(battery_msg)

    return battery_states_array_msg

def PowerStatesToMsg(power_state: robot_state_pb2.PowerState,
                     spot_wrapper: SpotWrapper) -> PowerState:
    """Maps power state data from robot state proto to ROS PowerState message

    Args:
        power_state: PowerState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        spot_msgs/PowerState ROS message
    """
    power_state_msg = PowerState()
    local_time = spot_wrapper.robotToLocalTime(power_state.timestamp)
    power_state_msg.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
    power_state_msg.motor_power_state = power_state.motor_power_state
    power_state_msg.shore_power_state = power_state.shore_power_state
    power_state_msg.locomotion_charge_percentage = power_state.locomotion_charge_percentage.value
    power_state_msg.locomotion_estimated_runtime = ROSDuration(sec=power_state.locomotion_estimated_runtime.seconds, nanosec=power_state.locomotion_estimated_runtime.nanos)
    return power_state_msg

def getBehaviorFaults(behavior_faults: service_fault_pb2.ServiceFault,
                      spot_wrapper: SpotWrapper) -> List[BehaviorFault]:
    """Helper function to strip out behavior faults into a list

    Args:
        behavior_faults: List of ServiceFault
        spot_wrapper: A SpotWrapper object
    Returns:
        List of BehaviorFault messages
    """
    faults = []

    for fault in behavior_faults:
        new_fault = BehaviorFault()
        new_fault.behavior_fault_id = fault.behavior_fault_id
        local_time = spot_wrapper.robotToLocalTime(fault.onset_timestamp)
        new_fault.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
        new_fault.cause = fault.cause
        new_fault.status = fault.status
        faults.append(new_fault)

    return faults

def getSystemFaults(system_faults: service_fault_pb2.ServiceFault,
                    spot_wrapper: SpotWrapper) -> List[SystemFault]:
    """Helper function to strip out system faults into a list

    Args:
        system_faults: List of SystemFault
        spot_wrapper: A SpotWrapper object
    Returns:
        List of SystemFault messages
    """
    faults = []

    for fault in system_faults:
        new_fault = SystemFault()
        new_fault.name = fault.name
        local_time = spot_wrapper.robotToLocalTime(fault.onset_timestamp)
        new_fault.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
        new_fault.duration = ROSDuration(sec=fault.duration.seconds, nanosec=fault.duration.nanos)
        new_fault.code = fault.code
        new_fault.uid = fault.uid
        new_fault.error_message = fault.error_message

        for att in fault.attributes:
            new_fault.attributes.append(att)

        new_fault.severity = fault.severity
        faults.append(new_fault)

    return faults

def SystemFaultsToMsg(system_fault_state: robot_state_pb2.SystemFaultState,
                      spot_wrapper: SpotWrapper) -> SystemFaultState:
    """Maps system fault data from robot state proto to ROS SystemFaultState message

    Args:
        system_fault_state: SystemFaultState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        slot_msgs/SystemFaultState ROS message
    """
    system_fault_state_msg = SystemFaultState()
    system_fault_state_msg.faults = getSystemFaults(system_fault_state.faults, spot_wrapper)
    system_fault_state_msg.historical_faults = getSystemFaults(system_fault_state.historical_faults, spot_wrapper)
    return system_fault_state_msg

def BehaviorFaultsToMsg(behavior_fault_state: robot_state_pb2.BehaviorFaultState,
                        spot_wrapper: SpotWrapper) -> BehaviorFaultState:
    """Maps behavior fault data from robot state proto to ROS BehaviorFaultState message

    Args:
        behavior_fault_state: BehaviorFaultState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        BehaviorFaultState message
    """
    behavior_fault_state_msg = BehaviorFaultState()
    behavior_fault_state_msg.faults = getBehaviorFaults(behavior_fault_state.faults, spot_wrapper)
    return behavior_fault_state_msg

def ManipulatorStatesToMsg(manipulator_state: robot_state_pb2.ManipulatorState,
                           spot_wrapper: SpotWrapper) -> ManipulatorState:
    """Maps manipulator state data from robot state proto to ROS ManipulatorState message

    Args:
        manipulator_state: ManipulatorState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        spot_msgs/ManipulatorState ROS message
    """
    if manipulator_state is None:
        return ManipulatorState()
    manipulator_state_msg = ManipulatorState()
    manipulator_state_msg.gripper_open_percentage = manipulator_state.gripper_open_percentage
    manipulator_state_msg.is_gripper_holding_item = manipulator_state.is_gripper_holding_item
    manipulator_state_msg.estimated_end_effector_force_in_hand.x = manipulator_state.estimated_end_effector_force_in_hand.x
    manipulator_state_msg.estimated_end_effector_force_in_hand.y = manipulator_state.estimated_end_effector_force_in_hand.y
    manipulator_state_msg.estimated_end_effector_force_in_hand.z = manipulator_state.estimated_end_effector_force_in_hand.z
    manipulator_state_msg.stow_state = manipulator_state.stow_state
    # manipulator_state_msg.velocity_of_hand_in_vision = manipulator_state.velocity_of_hand_in_vision
    # manipulator_state_msg.velocity_of_hand_in_odom = manipulator_state.velocity_of_hand_in_odom
    manipulator_state_msg.carry_state = manipulator_state.carry_state
    return manipulator_state_msg