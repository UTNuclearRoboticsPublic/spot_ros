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

from typing import List, Tuple
import rclpy.time

from .spot_wrapper import SpotWrapper

from builtin_interfaces.msg import Time as ROSTime
from builtin_interfaces.msg import Duration as ROSDuration
from geometry_msgs.msg import PoseWithCovariance, TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from spot_msgs.msg import FootState, FootStateArray
from spot_msgs.msg import EStopState, EStopStateArray
from spot_msgs.msg import WiFiState
from spot_msgs.msg import PowerState
from spot_msgs.msg import BehaviorFault, BehaviorFaultState
from spot_msgs.msg import SystemFault, SystemFaultState
from spot_msgs.msg import BatteryState, BatteryStateArray

from bosdyn.api import image_pb2, robot_state_pb2, service_fault_pb2
from bosdyn.client.math_helpers import SE3Pose
from bosdyn.client.frame_helpers import get_odom_tform_body, get_vision_tform_body

'''Dictionaries for mapping BD joint names to more friendly names'''
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
    'arm0.elo0': 'arm0_elbow_pitch',
    'arm0.elo1': 'arm0_elbow_roll',
    'arm0.wr0': 'arm0_wrist_pitch',
    'arm0.wr1': 'arm0_wrist_roll',
    'arm0.f1x': 'arm0_fingers'
}

friendly_joint_names = dict(body_joint_names, **arm_joint_names)

def populateTransformStamped(time: rclpy.time.Time,
                             parent_frame: str,
                             child_frame: str,
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
    if data.shot.image.format == image_pb2.Image.FORMAT_RAW:
        # One byte per pixel.
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
            image_msg.encoding = "mono8"
            image_msg.is_bigendian = True
            image_msg.step = data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Three bytes per pixel.
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = True
            image_msg.step = 3 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Four bytes per pixel.
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGBA_U8:
            image_msg.encoding = "rgba8"
            image_msg.is_bigendian = True
            image_msg.step = 4 * data.shot.image.cols
            image_msg.data = data.shot.image.data

        # Little-endian uint16 z-distance from camera (mm).
        if data.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            image_msg.encoding = "mono16"
            image_msg.is_bigendian = False
            image_msg.step = 2 * data.shot.image.cols
            image_msg.data = data.shot.image.data

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

def GetJointStatesFromState(kinematic_state: robot_state_pb2.KinematicState,
                            spot_wrapper: SpotWrapper) -> JointState:
    """Maps joint state data from robot state proto to ROS JointState message

    Args:
        kinematic_state: KinematicState proto
        spot_wrapper: A SpotWrapper object
    Returns:
        sensor_msgs/JointState ROS message
    """
    joint_state_msg = JointState()
    local_time = spot_wrapper.robotToLocalTime(kinematic_state.acquisition_timestamp)
    joint_state_msg.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)

    for joint in kinematic_state.joint_states:
        joint_state_msg.name.append(friendly_joint_names.get(joint.name, "ERROR"))
        joint_state_msg.position.append(joint.position.value)
        joint_state_msg.velocity.append(joint.velocity.value)
        joint_state_msg.effort.append(joint.load.value)

    return joint_state_msg

def GetEStopStatesFromState(estop_states: robot_state_pb2.EStopState,
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

def GetFeetFromState(foot_states: robot_state_pb2.FootState) -> FootStateArray:
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
            tf_msg.transforms.append(new_tf)

    return tf_msg

def GetBatteryStatesFromState(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> BatteryStateArray:
    """Maps battery state data from robot state proto to ROS BatteryStateArray message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        BatteryStateArray message
    """
    battery_states_array_msg = BatteryStateArray()
    for battery in state.battery_states:
        battery_msg = BatteryState()
        local_time = spot_wrapper.robotToLocalTime(battery.timestamp)
        battery_msg.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)

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

def GetPowerStatesFromState(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> PowerState:
    """Maps power state data from robot state proto to ROS PowerState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        PowerState message
    """
    power_state_msg = PowerState()
    local_time = spot_wrapper.robotToLocalTime(state.power_state.timestamp)
    power_state_msg.header.stamp = ROSTime(sec=local_time.seconds, nanosec=local_time.nanos)
    power_state_msg.motor_power_state = state.power_state.motor_power_state
    power_state_msg.shore_power_state = state.power_state.shore_power_state
    power_state_msg.locomotion_charge_percentage = state.power_state.locomotion_charge_percentage.value
    power_state_msg.locomotion_estimated_runtime = ROSDuration(sec=state.power_state.locomotion_estimated_runtime.seconds, nanosec=state.power_state.locomotion_estimated_runtime.nanos)
    return power_state_msg

def getBehaviorFaults(behavior_faults: service_fault_pb2.ServiceFault, spot_wrapper: SpotWrapper) -> List[BehaviorFault]:
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

def getSystemFaults(system_faults: service_fault_pb2.ServiceFault, spot_wrapper: SpotWrapper) -> List[SystemFault]:
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

def GetSystemFaultsFromState(state: robot_state_pb2.RobotState, spot_wrapper: SpotWrapper) -> SystemFaultState:
    """Maps system fault data from robot state proto to ROS SystemFaultState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        SystemFaultState message
    """
    system_fault_state_msg = SystemFaultState()
    system_fault_state_msg.faults = getSystemFaults(state.system_fault_state.faults, spot_wrapper)
    system_fault_state_msg.historical_faults = getSystemFaults(state.system_fault_state.historical_faults, spot_wrapper)
    return system_fault_state_msg

def getBehaviorFaultsFromState(state, spot_wrapper: SpotWrapper) -> BehaviorFaultState:
    """Maps behavior fault data from robot state proto to ROS BehaviorFaultState message

    Args:
        data: Robot State proto
        spot_wrapper: A SpotWrapper object
    Returns:
        BehaviorFaultState message
    """
    behavior_fault_state_msg = BehaviorFaultState()
    behavior_fault_state_msg.faults = getBehaviorFaults(state.behavior_fault_state.faults, spot_wrapper)
    return behavior_fault_state_msg
