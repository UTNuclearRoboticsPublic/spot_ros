############################################################################################
#      Title     : spot_ros.py
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

from typing import List, Text
import threading
import yaml
import time as pyTime
import math

import rclpy.utilities
from rclpy.node import Node
from rclpy.time import Time
import rclpy.action
import rclpy.callback_groups
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger, SetBool

from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import image_pb2, geometry_pb2, trajectory_pb2
from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.client import math_helpers

import functools
import tf2_ros

from spot_msgs.msg import LeaseArray, LeaseResource
from spot_msgs.msg import FootStateArray
from spot_msgs.msg import EStopStateArray
from spot_msgs.msg import WiFiState
from spot_msgs.msg import PowerState
from spot_msgs.msg import BehaviorFaultState
from spot_msgs.msg import SystemFaultState
from spot_msgs.msg import BatteryStateArray
from spot_msgs.msg import Feedback
from spot_msgs.msg import MobilityParams
from spot_msgs.msg import ManipulatorState
from spot_msgs.action import NavigateTo, Trajectory

from spot_msgs.srv import Dock, ClearBehaviorFault, ListGraph, SetLocomotion, SetVelocity
from spot_msgs.srv import ArmJointMovement, GripperAngleMove, ArmForceTrajectory, HandPose
    
from .ros_helpers import *

class SpotROS(Node):
    """Parent class for using the wrapper.  Defines all callbacks and keeps the wrapper alive"""

    """ Inner class for managing camera publishing """
    class CameraPubs():
        def __init__(self, parent, namespace: str):
            self.image_pub = parent.create_publisher(Image, '~/' + namespace + '/image', 1)
            self.info_pub = parent.create_publisher(CameraInfo, '~/' + namespace+'/camera_info', 1)
            self.spot_wrapper = parent.spot_wrapper

        def process_data(self, data):
            if self.image_pub.get_subscription_count() > 0:
                image_msg, camera_info_msg, _ = getImageMsg(data, self.spot_wrapper)
                self.image_pub.publish(image_msg)
                # TODO: use latching for the camera info publisher so we don't have to constantly republish it
                self.info_pub.publish(camera_info_msg)

    def __init__(self):
        super().__init__('spot_driver')

        self.spot_wrapper = None
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.status_timer = None

        pub_period = 0.1
        self.status_timer = self.create_timer(pub_period, self.publishStatus)
        self.sensors_timer = self.create_timer(pub_period, self.publishSensors)

        """ ROS Parameters """
        status_rate_params = {'rates.status.'  + param for param in {'robot_state', 'lease'}}
        sensor_rate_params = {'rates.sensors.' + param for param in {'front_image', 'side_image', 'rear_image', 'hand_image'}}
        self.add_on_set_parameters_callback(
            functools.partial(self.parameters_callback,
                              status_rate_params=status_rate_params,
                              sensor_rate_params=sensor_rate_params))
        
        self.declare_parameter('username', 'default_value',
            ParameterDescriptor(description='Spot computer username.',
                                type=ParameterType.PARAMETER_STRING,
                                read_only=True))

        self.declare_parameter('password', 'default_value',
            ParameterDescriptor(description='Spot computer password.',
                                type=ParameterType.PARAMETER_STRING,
                                read_only=True))
        
        self.declare_parameter('hostname', 'default_value',
            ParameterDescriptor(description='Spot computer hostname.',
                                type=ParameterType.PARAMETER_STRING,
                                read_only=True))

        self.declare_parameter('estop_timeout', 9.0,
            ParameterDescriptor(description='The E-Stop engages if we lose connection for this long.',
                                type=ParameterType.PARAMETER_INTEGER,
                                floating_point_range=[FloatingPointRange(
                                    from_value=0.0, to_value=1.0e9, step=0.0)],
                                read_only=True))

        for name in status_rate_params:
            self.declare_parameter(name, 1.0,
                ParameterDescriptor(description='Publish rate for robot status topics.',
                                    type=ParameterType.PARAMETER_DOUBLE,
                                    floating_point_range=[FloatingPointRange(
                                        from_value=0.0, to_value=1.0e9, step=0.0)],
                                    read_only=True))
        
        for name in sensor_rate_params:
            self.declare_parameter(name, 1.5,
                ParameterDescriptor(description='Publish rate for sensor topics.',
                                    type=ParameterType.PARAMETER_DOUBLE,
                                    floating_point_range=[FloatingPointRange(
                                        from_value=0.0, to_value=1.0e9, step=0.0)],
                                    read_only=True))

        # Spot has 2 types of odometries: 'odom' and 'vision'
        # The former one is kinematic odometry and the second one is a combined odometry of vision and kinematics
        self.declare_parameter('odom_mode', 'odom',
            ParameterDescriptor(description='Selects pure kinematic odometry or fused vision and kinematic odometry.',
                                type=ParameterType.PARAMETER_STRING,
                                additional_constraints="'odom' or 'vision'",
                                read_only=True))

        self.declare_parameter('has_cam_payload', False,
            ParameterDescriptor(description='Set true if this robot features the Spot CAM payload.',
                                type=ParameterType.PARAMETER_BOOL,
                                read_only=True))

        self.declare_parameter('sounds', Text(''),
            ParameterDescriptor(description='Array of YAML files giving WAV sound files to load. Keys in the files are labels and values are the filepaths.',
                                type=ParameterType.PARAMETER_STRING_ARRAY,
                                read_only=True))

        self.declare_parameter('auto_claim', False,
            ParameterDescriptor(description='Automatically claim ownership of the robot on connection.',
                                type=ParameterType.PARAMETER_BOOL,
                                read_only=True))

        self.declare_parameter('auto_power_on', False,
            ParameterDescriptor(description='Automatically power on the robot on connection.',
                                type=ParameterType.PARAMETER_BOOL,
                                read_only=True))

        self.declare_parameter('auto_stand', False,
            ParameterDescriptor(description='Automatically stand up the robot on connection.',
                                type=ParameterType.PARAMETER_BOOL,
                                read_only=True))

    def __del__(self):
        if self.status_timer is not None:
            self.status_timer.destroy()

        if self.spot_wrapper is None:
            return

        if not self.spot_wrapper.is_connected:
            return

        if not self.spot_wrapper.is_sitting:
            print('Spot sitting down...')
            is_sitting, message = self.spot_wrapper.sit()
        
            if not is_sitting:
                print('Not shutting down because Spot cannot sit here! ' + message)
                return

        print('Shutting down ROS driver for Spot')
        self.spot_wrapper.disconnect()

    def RobotStateCB(self, _) -> None:
        """Callback for when the Spot Wrapper gets new robot state data."""
        state = self.spot_wrapper.robot_state

        if not state:
            return

        odom_mode = self.get_parameter('odom_mode').value
        
        ## joint states ##
        joint_state = JointStatesToMsg(state.kinematic_state, self.spot_wrapper)
        self.joint_state_pub.publish(joint_state)
        
        ## TF ##
        tf_msg = GetTFFromState(state.kinematic_state, self.spot_wrapper)
        
        if len(tf_msg.transforms) > 0:
            self.tf_broadcaster.sendTransform(tf_msg.transforms)
        
        # Odom Twist #
        twist_odom_msg = GetOdomTwistFromState(state.kinematic_state, self.spot_wrapper)
        self.odom_twist_pub.publish(twist_odom_msg)

        # Odom #
        odom_msg = GetOdomFromState(state.kinematic_state, self.spot_wrapper, odom_mode == 'vision')
        self.odom_pub.publish(odom_msg)
        
        # Feet #
        foot_array_msg = FeetStateToMsg(state.foot_state)
        self.feet_pub.publish(foot_array_msg)

        # EStop #
        estop_array_msg = EStopStatesToMsg(state.estop_states, self.spot_wrapper)
        self.estop_pub.publish(estop_array_msg)

        # WIFI #
        wifi_msg = GetWifiFromState(state.comms_states)
        self.wifi_pub.publish(wifi_msg)

        # Battery States #
        battery_states_array_msg = BatteryStatesToMsg(state.battery_states, self.spot_wrapper)
        self.battery_pub.publish(battery_states_array_msg)
        
        # Power State #
        power_state_msg = PowerStatesToMsg(state.power_state, self.spot_wrapper)
        self.power_pub.publish(power_state_msg)

        # System Faults #
        system_fault_state_msg = SystemFaultsToMsg(state.system_fault_state, self.spot_wrapper)
        self.system_faults_pub.publish(system_fault_state_msg)

        # Behavior Faults #
        behavior_fault_state_msg = BehaviorFaultsToMsg(state.behavior_fault_state, self.spot_wrapper)
        self.behavior_faults_pub.publish(behavior_fault_state_msg)

        manipulator_state_msg = ManipulatorStatesToMsg(state.manipulator_state, self.spot_wrapper)
        self.manipulator_state_pub.publish(manipulator_state_msg)

    def LeaseCB(self, _) -> None:
        """Callback for when the Spot Wrapper gets new lease data."""
        lease_array_msg = LeaseArray()
        lease_list = self.spot_wrapper.lease

        if not lease_list:
            return
        
        for resource in lease_list:
            new_resource = LeaseResource()
            new_resource.resource = resource.resource
            new_resource.lease.resource = resource.lease.resource
            new_resource.lease.epoch = resource.lease.epoch

            for seq in resource.lease.sequence:
                new_resource.lease.sequence.append(seq)

            new_resource.lease_owner.client_name = resource.lease_owner.client_name
            new_resource.lease_owner.user_name = resource.lease_owner.user_name

            lease_array_msg.resources.append(new_resource)

        self.lease_pub.publish(lease_array_msg)

    def FrontImageCB(self, _) -> None:
        """Callback for when the Spot Wrapper gets new front image data."""

        # [front left image, front right image, front left depth, front right depth]
        data = self.spot_wrapper.front_images

        if data and len(data) == 4:
            self.front_left_rgb_pub.process_data(data[0])
            self.front_right_rgb_pub.process_data(data[1])
            self.front_left_depth_pub.process_data(data[2])
            self.front_right_depth_pub.process_data(data[3])

    def SideImageCB(self, _) -> None:
        """Callback for when the Spot Wrapper gets new side image data."""

        # [left image, right image, left depth, right depth]
        data = self.spot_wrapper.side_images

        if data and len(data) == 4:
            self.left_rgb_pub.process_data(data[0])
            self.right_rgb_pub.process_data(data[1])
            self.left_depth_pub.process_data(data[2])
            self.right_depth_pub.process_data(data[3])

    def RearImageCB(self, _) -> None:
        """Callback for when the Spot Wrapper gets new rear image data."""

        # [image, depth]
        data = self.spot_wrapper.rear_images

        if data and len(data) == 2:
            self.back_rgb_pub.process_data(data[0])
            self.back_depth_pub.process_data(data[1])

    def HandImageCB(self, _) -> None:
        """Callback for when the Spot Wrapper gets new hand image data."""

        # Data order is non-deterministic, so we check the names before processing
        for image in self.spot_wrapper.hand_images:
            if image.source.name == "hand_image":
                self.hand_mono_rgb_pub.process_data(image)
            elif image.source.name == "hand_depth":
                self.hand_depth_pub.process_data(image)
            elif image.source.name == "hand_color_image":
                self.hand_rgb_pub.process_data(image)
            elif image.source.name == "hand_depth_in_hand_color_frame":
                self.hand_depth_in_color_pub.process_data(image)

        
    def handle_claim(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the claim service"""
        print('Debug 1')
        res.success, res.message = self.spot_wrapper.claim()
        return res

    def handle_release(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the release service"""
        res.success, res.message = self.spot_wrapper.release()
        return res

    def handle_stop(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the stop service"""
        resp = self.spot_wrapper.stop()
        return Trigger.Response(res[0], resp[1])

    def handle_self_right(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the self-right service"""
        resp = self.spot_wrapper.self_right()
        return Trigger.Response(resp[0], resp[1])

    def handle_sit(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the sit service"""
        res.success, res.message = self.spot_wrapper.sit()
        return res

    def handle_stand(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the stand service"""
        res.success, res.message = self.spot_wrapper.stand()
        return res

    def handle_dock(self, req: Dock.Request, res: Dock.Response) -> Dock.Response:
        """Dock the robot"""
        res.success, res.message = self.spot_wrapper.dock(req.dock_id)
        self.update_dock_state()
        return res

    def handle_undock(self, _, res: Trigger.Response) -> Trigger.Response:
        """Undock the robot"""
        res.success, res.message = self.spot_wrapper.undock()
        self.update_dock_state()
        return res

    def update_dock_state(self) -> None:
        """Get docking state of robot"""
        res = self.spot_wrapper.get_docking_state()
        self.dock_state_pub.publish(DockStateToMsg(res))

    def handle_power_on(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the power-on service"""
        res.success, res.message = self.spot_wrapper.power_on()
        if res.success:
            res.message = 'Powered on Spot robot ' + self.spot_wrapper.id.nickname
        return res

    def handle_safe_power_off(self, _) -> Trigger.Response:
        """ROS service handler for the safe-power-off service"""
        resp = self.spot_wrapper.safe_power_off()
        return Trigger.Response(resp[0], resp[1])

    def handle_estop_hard(self, _) -> Trigger.Response:
        """ROS service handler to hard-eStop the robot.  The robot will immediately cut power to the motors"""
        resp = self.spot_wrapper.assertEStop(True)
        return Trigger.Response(resp[0], resp[1])

    def handle_estop_soft(self, _) -> Trigger.Response:
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting
        power to the motors """
        resp = self.spot_wrapper.assertEStop(False)
        return Trigger.Response(resp[0], resp[1])

    def handle_estop_disengage(self, _) -> Trigger.Response:
        """ROS service handler to disengage the eStop on the robot."""
        resp = self.spot_wrapper.disengageEStop()
        return Trigger.Response(resp[0], resp[1])

    def handle_clear_behavior_fault(self, req) -> ClearBehaviorFault.Response:
        """ROS service handler for clearing behavior faults"""
        resp = self.spot_wrapper.clear_behavior_fault(req.id)
        return ClearBehaviorFault.Response(resp[0], resp[1])

    def handle_stair_mode(self, req) -> SetBool.Response:
        """ROS service handler to set a stair mode to the robot."""
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.stair_hint = req.data
            self.spot_wrapper.set_mobility_params(mobility_params)
            return SetBool.Response(True, 'Success')
        except Exception as e:
            return SetBool.Response(False, Text(e))

    def handle_locomotion_mode(self, req) -> SetLocomotion.Response:
        """ROS service handler to set locomotion mode"""
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.locomotion_hint = req.locomotion_mode
            self.spot_wrapper.set_mobility_params( mobility_params )
            return SetLocomotion.Response(True, 'Success')
        except Exception as e:
            return SetLocomotion.Response(False, Text(e))

    def handle_max_vel(self, req: SetVelocity.Request) -> SetVelocity.Response:
        """
        Handle a max_velocity service call. This will modify the mobility params to set a limit on the maximum
        velocity that the robot can move during motion commmands. This affects trajectory commands and velocity
        commands

        Args:
            req: SetVelocity.Request containing requested maximum velocity

        Returns: SetVelocity.Response
        """
        if (req.velocity_limit.linear.x >= 0.0 or
            req.velocity_limit.linear.y >= 0.0 or
            req.velocity_limit.linear.z >= 0.0):
            return SetVelocity.Response(False, 'Cannot set a non-positive velocity limit.')

        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.vel_limit.CopyFrom(
                SE2VelocityLimit(max_vel=math_helpers.SE2Velocity(req.velocity_limit.linear.x,
                                                                  req.velocity_limit.linear.y,
                                                                  req.velocity_limit.angular.z).to_proto()))
            self.spot_wrapper.set_mobility_params(mobility_params)
            return SetVelocity.Response(True, 'Success')
        except Exception as e:
            return SetVelocity.Response(False, e)

    def handle_trajectory(self, req: Trajectory.Goal) -> None:
        """ROS actionserver execution handler to handle receiving a request to move to a location"""
        if req.target_pose.header.frame_id != 'body':
            self.trajectory_server.set_aborted(
                Trajectory.Result(False, 'frame_id of target_pose must be \'body\''))
            return
        if req.duration.data.to_sec() <= 0:
            self.trajectory_server.set_aborted(Trajectory.Result(False, 'duration must be larger than 0'))
            return

        cmd_duration = rclpy.time.Duration(req.duration.data.secs, req.duration.data.nsecs)
        resp = self.spot_wrapper.trajectory_cmd(
                        goal_x=req.target_pose.pose.position.x,
                        goal_y=req.target_pose.pose.position.y,
                        goal_heading=math_helpers.Quat(
                            w=req.target_pose.pose.orientation.w,
                            x=req.target_pose.pose.orientation.x,
                            y=req.target_pose.pose.orientation.y,
                            z=req.target_pose.pose.orientation.z
                            ).to_yaw(),
                        cmd_duration=cmd_duration.to_sec(),
                        precise_position=req.precise_positioning,
                        )

        # Wait while robot performs the trajectory
        rate = self.create_rate(10)
        start_time = self.get_clock().now()
        while (rclpy.ok() and self.trajectory_server.is_active()):
            if self.trajectory_server.is_preempt_requested():
                self.trajectory_server.set_preempted(Trajectory.Feedback(False, "Preempted"))
                self.spot_wrapper.stop()
                return
            elif self.spot_wrapper.at_goal:
                self.trajectory_server.set_succeeded(Trajectory.Result(resp[0], resp[1]))
                return
            elif self.spot_wrapper.near_goal:
                if self.spot_wrapper._last_trajectory_command_precise:
                    self.trajectory_server.publish_feedback(
                        Trajectory.Feedback("Near goal, performing precise adjustments"))
                else:
                    self.trajectory_server.publish_feedback(Trajectory.Feedback("Near goal"))
            else:
                self.trajectory_server.publish_feedback(Trajectory.Feedback("Moving to goal"))

            # check for timeout
            if (self.get_clock().now() - start_time > cmd_duration):
                # the action has timed out. abort.
                self.trajectory_server.set_aborted(
                    Trajectory.Result(False, "Failed to reach goal, timed out"))
                return

            rate.sleep()

        # We timed out
        self.trajectory_server.set_aborted(Trajectory.Result(False, "Failed to reach goal"))

    def cmdVelCallback(self, data) -> None:
        """Callback for cmd_vel command"""
        self.spot_wrapper.velocity_cmd(data.linear.x, data.linear.y, data.angular.z)

    def bodyPoseCallback(self, data) -> None:
        """Callback for cmd_vel command"""
        q = data.orientation
        position = geometry_pb2.Vec3(z=data.position.z)
        pose = geometry_pb2.SE3Pose(position=position, rotation=q)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        traj = trajectory_pb2.SE3Trajectory(points=[point])
        body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

        mobility_params = self.spot_wrapper.get_mobility_params()
        mobility_params.body_control.CopyFrom(body_control)
        self.spot_wrapper.set_mobility_params(mobility_params)

    def handle_list_graph(self, upload_path) -> ListGraph.Response:
        """ROS service handler for listing graph_nav waypoint_ids"""
        resp = self.spot_wrapper.list_graph(upload_path)
        return ListGraph.Response(resp)

    def handle_navigate_to_feedback(self) -> None:
        """Thread function to send navigate_to feedback"""
        rate = self.create_rate(10)
        while rclpy.ok() and self.run_navigate_to:
            localization_state = self.spot_wrapper._graph_nav_client.get_localization_state()
            if localization_state.localization.waypoint_id:
                self.navigate_as.publish_feedback(NavigateTo.Feedback(localization_state.localization.waypoint_id))
            rate.sleep()

    def handle_navigate_to(self, msg) -> None:
        """ROS service handler to run mission of the robot.  The robot will replay a mission"""
        # create thread to periodically publish feedback
        feedback_thread = threading.Thread(target = self.handle_navigate_to_feedback, args = ())
        self.run_navigate_to = True
        feedback_thread.start()
        # run navigate_to
        resp = self.spot_wrapper.navigate_to(upload_path = msg.upload_path,
                                             navigate_to = msg.navigate_to,
                                             initial_localization_fiducial = msg.initial_localization_fiducial,
                                             initial_localization_waypoint = msg.initial_localization_waypoint)
        self.run_navigate_to = False
        feedback_thread.join()

        # check status
        if resp[0]:
            self.navigate_as.set_succeeded(NavigateTo.Result(resp[0], resp[1]))
        else:
            self.navigate_as.set_aborted(NavigateTo.Result(resp[0], resp[1]))

    def populate_camera_static_transforms(self,
                                          image_data: image_pb2.ImageResponse,
                                          existing_transforms: List[TransformStamped]) -> List[TransformStamped]:
        """Check data received from one of the image tasks and use the transform snapshot to extract the camera frame
        transforms. These are the transforms from body->frontleft->frontleft_fisheye, for example. These transforms
        never change, but they may be calibrated slightly differently for each robot so we need to generate the
        transforms at runtime.

        Args:
            image_data: ImageResponse protobuf data from the wrapper
        """

        # We exclude the odometry frames from static transforms since they are not static. We can ignore the body
        # frame because it is a child of odom or vision depending on the odom_mode, and will be published
        # by the non-static transform publishing that is done by the state callback
        excluded_child_frames = {'odom', 'vision', 'arm0.link_wr1', 'hand_color_image_sensor'}
        all_tfs_from_data = image_data.shot.transforms_snapshot.child_to_parent_edge_map
        existing_pairs = [(transform.header.frame_id, transform.child_frame_id) for transform in existing_transforms]

        tfs_to_add = {k:v for (k,v) in all_tfs_from_data.items()
            if k not in excluded_child_frames and (v.parent_frame_name, k) not in existing_pairs
            and len(v.parent_frame_name) != 0}

        # tf: FrameTreeSnapshot.ChildToParentEdgeMapEntry
        #    key: Text
        #    value: FrameTreeSnapshot.ParentEdge
        #       parent_frame_name: Text
        #       parent_tform_child: bosdyn.client.math_helpers.SE3Pose
        output = existing_transforms
        for k,v in tfs_to_add.items():
            local_time = self.spot_wrapper.robotToLocalTime(image_data.shot.acquisition_time)
            tf_time = Time(seconds=local_time.seconds, nanoseconds=local_time.nanos)
            static_tf = populateTransformStamped(tf_time,
                                                 v.parent_frame_name,
                                                 k,
                                                 v.parent_tform_child)
            output.append(static_tf)

        return output

    def parameters_callback(self, params, status_rate_params, sensor_rate_params) -> SetParametersResult:

        for p in params:
            if p.name == 'odom_mode':
                allowed = {'odom','vision'}
                if p.value not in allowed:
                    return SetParametersResult(
                        successful=False,
                        reason="Parameter 'odom_mode' must take value 'odom' or 'vision'.")
            elif p.name in status_rate_params:
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="Parameter rates." + p.name + " must be positive.")
            elif p.name in sensor_rate_params:
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="Parameter rates." + p.name + " must be positive.")
        
        return SetParametersResult(successful=True)

    # Arm ######
    def handle_arm_stow(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler to command the arm to stow, home position"""
        res.success, res.message = self.spot_wrapper.arm_stow()
        return res

    def handle_arm_unstow(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler to command the arm to unstow, joints are all zeros"""
        res.success, res.message = self.spot_wrapper.arm_unstow()
        return res

    def handle_gripper_open(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler to open the gripper"""
        res.success, res.message = self.spot_wrapper.gripper_open()
        return res

    def handle_gripper_angle_open(self, req: GripperAngleMove.Request, res: GripperAngleMove.Response) -> GripperAngleMove.Response:
        """ROS service handler to open the gripper at an angle"""
        res.success, res.message = self.spot_wrapper.gripper_angle_open(gripper_ang=req.gripper_angle)
        return res

    def handle_gripper_close(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler to close the gripper"""
        res.success, res.message = self.spot_wrapper.gripper_close()
        return res

    def handle_arm_carry(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler to put arm in carry mode"""
        res.success, res.message = self.spot_wrapper.arm_carry()
        return res

    # def handle_arm_joint_move(self, _, req: ArmJointMovement.Request, res: ArmJointMovement.Response) -> ArmJointMovement.Response:
    #     """ROS service handler to send joint movement to the arm to execute"""
    #     resp = self.spot_wrapper.arm_joint_move(joint_targets=req.joint_target)
    #     return ArmJointMovement.Response(resp[0], resp[1])

    def handle_force_trajectory(self, _, req: ArmForceTrajectory.Request, res: ArmForceTrajectory.Response) -> ArmForceTrajectory.Response:
        """ROS service handler to send a force trajectory up or down a vertical force"""
        resp = self.spot_wrapper.force_trajectory(data=req)
        return ArmForceTrajectory.Response(resp[0], resp[1])

    # def handle_hand_pose(self, _, req: HandPose.Request, res: HandPose.Response) -> HandPose.Response:
    #     """ROS service to give a position to the gripper"""
    #     resp = self.spot_wrapper.hand_pose(pose_points=req.pose_point)
    #     return HandPose.Response(resp[0], resp[1])

######

    def connect(self) -> bool:
        """
            Main function for the SpotROS class.
            Gets config from ROS and initializes the wrapper.
            Holds lease from wrapper and updates all async tasks at the ROS rate
        """

        """Dictionary listing what callback to use for what data task"""
        self.get_logger().info("Setting sensor callbacks")
        callbacks = {}
        callbacks["robot_state"] = self.RobotStateCB
        callbacks["lease"]       = self.LeaseCB
        callbacks["front_image"] = self.FrontImageCB
        callbacks["side_image"]  = self.SideImageCB
        callbacks["rear_image"]  = self.RearImageCB
        callbacks["hand_image"]  = self.HandImageCB

        has_cam_payload = self.get_parameter('has_cam_payload').value

        # Connect to the robot
        self.spot_wrapper = SpotWrapper(has_cam_payload)

        # Dictionary of all param values in the 'rates' namespace
        rates_dict = {name: value.value for name, value in self.get_parameters_by_prefix('rates').items() }
        self.get_logger().info(f"Rates: {rates_dict}")

        # Verify connection
        if self.spot_wrapper.connect(self.get_logger(),
                                     #self.get_parameter('username').value, 
                                     #self.get_parameter('password').value,
                                     self.get_parameter('hostname').value,
                                     rates_dict,
                                     callbacks):
            self.get_logger().info('Connected to Spot ')# + self.spot_wrapper.id.nickname + '...')
        else:
            self.get_logger().fatal('Failed to launch ROS driver!')
            return False

        # Startup routine per parameter configuration
        if self.get_parameter('auto_claim').value:
            if self.spot_wrapper.claim():
                self.get_logger().info('Claimed lease on Spot robot ' + self.spot_wrapper.id.nickname + '...')
                if self.get_parameter('auto_power_on').value:
                    self.get_logger().info('Spot powered on...')
                    if self.spot_wrapper.power_on():
                        if self.get_parameter('auto_stand').value:
                            self.get_logger().info('Spot standing up...')
                            pyTime.sleep(1.0)
                            self.spot_wrapper.stand()

        ### Set up ROS interfaces
        ## Camera publishers
        # RGB Images
        self.front_left_rgb_pub = self.CameraPubs(self, 'rgb/frontleft')
        self.front_right_rgb_pub = self.CameraPubs(self, 'rgb/frontright')
        self.left_rgb_pub = self.CameraPubs(self, 'rgb/left')
        self.right_rgb_pub = self.CameraPubs(self, 'rgb/right')
        self.back_rgb_pub = self.CameraPubs(self, 'rgb/back')
        self.hand_rgb_pub = self.CameraPubs(self, 'rgb/hand_color')
        self.hand_mono_rgb_pub = self.CameraPubs(self, 'rgb/hand_mono')

        # Depth Images
        self.front_left_depth_pub = self.CameraPubs(self, 'depth/frontleft')
        self.front_right_depth_pub = self.CameraPubs(self, 'depth/frontright')
        self.left_depth_pub = self.CameraPubs(self, 'depth/left')
        self.right_depth_pub = self.CameraPubs(self, 'depth/right')
        self.back_depth_pub = self.CameraPubs(self, 'depth/back')
        self.hand_depth_pub = self.CameraPubs(self, 'depth/hand')
        self.hand_depth_in_color_pub = self.CameraPubs(self, 'depth/hand/depth_in_color')

        ## Status Publishers
        # QoS to use for latched publishers
        latched_qos = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.dock_state_pub = self.create_publisher(DockState, '~/status/dock_state', qos_profile=latched_qos)
        self.lease_pub = self.create_publisher(LeaseArray, '~/status/leases', 1)
        self.odom_twist_pub = self.create_publisher(TwistWithCovarianceStamped, '~/odometry/twist', 1)
        self.odom_pub = self.create_publisher(Odometry, '~/odometry', 10)
        self.feet_pub = self.create_publisher(FootStateArray, '~/status/feet', 10)
        self.estop_pub = self.create_publisher(EStopStateArray, '~/status/estop', 1)
        self.wifi_pub = self.create_publisher(WiFiState, '~/status/wifi', qos_profile=latched_qos)
        self.power_pub = self.create_publisher(PowerState, '~/status/power_state', 1)
        self.battery_pub = self.create_publisher(BatteryStateArray, '~/status/battery_states', 1)
        self.behavior_faults_pub = self.create_publisher(BehaviorFaultState, '~/status/behavior_faults', 10)
        self.system_faults_pub = self.create_publisher(SystemFaultState, '~/status/system_faults', 10)
        self.mobility_params_pub = self.create_publisher(MobilityParams, '~/status/mobility_params', 1)
        self.feedback_pub = self.create_publisher(Feedback, '~/status/feedback', qos_profile=latched_qos)
        self.manipulator_state_pub = self.create_publisher(ManipulatorState, '~/status/manipulator_states', qos_profile=latched_qos)

        self.create_subscription(Twist, '~/cmd_vel', self.cmdVelCallback, 10)
        self.create_subscription(Pose, '~/body_pose', self.bodyPoseCallback, 10)

        srv_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "~/claim", self.handle_claim, callback_group=srv_group)
        self.create_service(Trigger, "~/release", self.handle_release, callback_group=srv_group)
        self.create_service(Trigger, "~/stop", self.handle_stop, callback_group=srv_group)
        self.create_service(Trigger, "~/self_right", self.handle_self_right, callback_group=srv_group)
        self.create_service(Trigger, "~/sit", self.handle_sit, callback_group=srv_group)
        self.create_service(Trigger, "~/stand", self.handle_stand, callback_group=srv_group)
        self.create_service(Trigger, "~/power_on", self.handle_power_on, callback_group=srv_group)
        self.create_service(Trigger, "~/power_off", self.handle_safe_power_off, callback_group=srv_group)

        self.create_service(Trigger, "~/estop/hard", self.handle_estop_hard, callback_group=srv_group)
        self.create_service(Trigger, "~/estop/gentle", self.handle_estop_soft, callback_group=srv_group)
        self.create_service(Trigger, "~/estop/release", self.handle_estop_disengage, callback_group=srv_group)

        self.create_service(SetBool, "~/stair_mode", self.handle_stair_mode, callback_group=srv_group)
        self.create_service(SetLocomotion, "~/locomotion_mode", self.handle_locomotion_mode, callback_group=srv_group)
        self.create_service(SetVelocity, "~/max_velocity", self.handle_max_vel, callback_group=srv_group)
        self.create_service(ClearBehaviorFault, "~/clear_behavior_fault", self.handle_clear_behavior_fault, callback_group=srv_group)

        self.create_service(ListGraph, "~/list_graph", self.handle_list_graph, callback_group=srv_group)

        # Docking
        self.create_service(Dock, '~/dock', self.handle_dock, callback_group=srv_group)
        self.create_service(Trigger, '~/undock', self.handle_undock, callback_group=srv_group)

        # Arm
        self.create_service(Trigger, '~/arm/stow', self.handle_arm_stow, callback_group=srv_group)
        self.create_service(Trigger, '~/arm/unstow', self.handle_arm_unstow, callback_group=srv_group)
        self.create_service(Trigger, '~/arm/carry', self.handle_arm_carry, callback_group=srv_group)
        self.create_service(Trigger, '~/arm/gripper_open', self.handle_gripper_open, callback_group=srv_group)
        self.create_service(Trigger, '~/arm/gripper_close', self.handle_gripper_close, callback_group=srv_group)
        self.create_service(GripperAngleMove, '~/arm/gripper_angle_open', self.handle_gripper_angle_open, callback_group=srv_group)

        self._navigate_to_server = rclpy.action.ActionServer(
                self,
                NavigateTo,
                '~/navigate_to',
                execute_callback=self.handle_navigate_to,
                callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        
        self._trajectory_server = rclpy.action.ActionServer(
                self,
                Trajectory,
                '~/trajectory',
                execute_callback=self.handle_trajectory,
                callback_group=rclpy.callback_groups.ReentrantCallbackGroup())

        # populate the static transforms for the various robot cameras
        
        def populate_static_transforms() -> None:

            # TODO: Handle the case of no arm (i.e. arm images will never appear)
            while not (self.spot_wrapper.front_images and len(self.spot_wrapper.front_images) == 4) or\
                  not (self.spot_wrapper.side_images and len(self.spot_wrapper.side_images) == 4) or\
                  not (self.spot_wrapper.rear_images and len(self.spot_wrapper.rear_images) == 2) or\
                  not (self.spot_wrapper.hand_images and len(self.spot_wrapper.hand_images) == 4) and\
                  rclpy.utilities.ok():
                self.spot_wrapper.updateSensorTasks()

            static_tfs = []

            data = self.spot_wrapper.front_images
            static_tfs = self.populate_camera_static_transforms(data[0], static_tfs)
            static_tfs = self.populate_camera_static_transforms(data[1], static_tfs)
            static_tfs = self.populate_camera_static_transforms(data[2], static_tfs)
            static_tfs = self.populate_camera_static_transforms(data[3], static_tfs)

            data = self.spot_wrapper.side_images
            static_tfs = self.populate_camera_static_transforms(data[0], static_tfs)
            static_tfs = self.populate_camera_static_transforms(data[1], static_tfs)
            static_tfs = self.populate_camera_static_transforms(data[2], static_tfs)
            static_tfs = self.populate_camera_static_transforms(data[3], static_tfs)

            data = self.spot_wrapper.rear_images
            static_tfs = self.populate_camera_static_transforms(data[0], static_tfs)
            static_tfs = self.populate_camera_static_transforms(data[1], static_tfs)

            if self.spot_wrapper.hand_images is not None:
                data = self.spot_wrapper.hand_images
                static_tfs = self.populate_camera_static_transforms(data[0], static_tfs)
                static_tfs = self.populate_camera_static_transforms(data[1], static_tfs)
                static_tfs = self.populate_camera_static_transforms(data[2], static_tfs)
                static_tfs = self.populate_camera_static_transforms(data[3], static_tfs)

            self.static_broadcaster.sendTransform(static_tfs)                
        
        populate_static_transforms()

        self.get_logger().info('Spot driver startup complete.')

        # Publish initial dock state. Wait for first response
        while self.spot_wrapper.get_docking_state().status == DockState.DOCK_STATUS_UNKNOWN:
            pass
        self.update_dock_state()

        return True

    def loadSounds(self):
        sounds_manifests = self.get_parameter('sounds_manifests').value

        if not sounds_manifests:
            return True

        for manifest in sounds_manifests:
            try:
                file = open(manifest, "r")

                try:
                    sound_names = yaml.safe_load(file)
                    
                    if not sound_names:
                        self.get_logger().warn('Opened sounds manifest file {}, but no contents found.'.format(manifest))

                    for name in sound_names:
                        try:
                            with open(name+'.wav', 'rb') as wav_file:
                                wav_data = wav_file.read()
                        except IOError as err:
                            self.get_logger().error(Text(err))
                            continue

                        self.spot_wrapper.load_sound(name, wav_data)
                except yaml.YAMLError as err:
                    self.get_logger().error(Text(err))
            except IOError as err:
                self.get_logger().error(Text(err))

        return True

    def publishSensors(self):
        if self.spot_wrapper is None:
            return

        if not self.spot_wrapper.is_connected:
            return

        # call sensor periodic tasks
        self.spot_wrapper.updateSensorTasks()

    def publishStatus(self):
        if self.spot_wrapper is None:
            return

        if not self.spot_wrapper.is_connected:
            return

        # call state periodic tasks
        self.spot_wrapper.updateStatusTasks()

        # publish robot feedback state
        feedback_msg = Feedback()
        feedback_msg.standing = self.spot_wrapper.is_standing
        feedback_msg.sitting = self.spot_wrapper.is_sitting
        feedback_msg.moving = self.spot_wrapper.is_moving
        # id = self.spot_wrapper.id
        id = None
        if id:
            feedback_msg.serial_number = id.serial_number
            feedback_msg.species = id.species
            feedback_msg.version = id.version
            feedback_msg.nickname = id.nickname
            feedback_msg.computer_serial_number = id.computer_serial_number
        self.feedback_pub.publish(feedback_msg)

        # publish mobility state
        mobility_params_msg = MobilityParams()
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params_msg.body_control.position.x = \
                    mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.x
            mobility_params_msg.body_control.position.y = \
                    mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.y
            mobility_params_msg.body_control.position.z = \
                    mobility_params.body_control.base_offset_rt_footprint.points[0].pose.position.z
            mobility_params_msg.body_control.orientation.x = \
                    mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.x
            mobility_params_msg.body_control.orientation.y = \
                    mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.y
            mobility_params_msg.body_control.orientation.z = \
                    mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.z
            mobility_params_msg.body_control.orientation.w = \
                    mobility_params.body_control.base_offset_rt_footprint.points[0].pose.rotation.w
            mobility_params_msg.locomotion_hint = mobility_params.locomotion_hint
            mobility_params_msg.stair_hint = mobility_params.stair_hint
        except Exception as e:
            self.get_logger().error('Error:{}'.format(e))
            pass
        self.mobility_params_pub.publish(mobility_params_msg)
