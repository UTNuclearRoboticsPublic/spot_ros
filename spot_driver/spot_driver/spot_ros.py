############################################################################################
#      Title     : spot_ros.py
#      Project   : spot_ros
############################################################################################

from typing import List, Text
import threading
import yaml
import time as pyTime
import math

import rclpy.duration
import rclpy.utilities
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action.server import ServerGoalHandle
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, qos_profile_sensor_data

from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, SetBool

from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import geometry_pb2
from bosdyn.api.geometry_pb2 import SE2VelocityLimit
from bosdyn.api.payload_pb2 import Payload, MountFrameName
from bosdyn.client import math_helpers
from bosdyn.geometry import to_euler_zxy

from .spot_lease_manager import SpotLeaseManager
from .spot_body_wrapper import SpotBodyWrapper
from .type_hint_helpers import *
from .ros_helpers import *

import functools
import tf2_ros
from tf2_geometry_msgs import PoseStamped

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
from spot_msgs.action import NavigateTo, WalkTo

from spot_msgs.srv import (Dock, ClearBehaviorFault, ListGraph, SetLocomotion, SetVelocity,
                           GestureSequence, TogglePayload, RegisterPayload)

class SpotROS(Node):
    """Parent class for using the wrapper.  Defines all callbacks and keeps the wrapper alive"""

    def __init__(self):
        super().__init__('spot_driver')

        self.spot_wrapper = None
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.status_timer = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        """ ROS Parameters """
        status_rate_params = {f'rates.status.{param}'  for param in {'robot_state', 'lease', 'feedback'}}
        sensor_rate_params = {f'rates.sensors.{param}' for param in {'point_cloud'}}

        default_rates_hz = {
            'rates.status.robot_state' : 10.0,
            'rates.status.lease'       :  1.0,
            'rates.status.feedback'    : 10.0,
            'rates.sensors.point_cloud': 10.0
        }

        self.add_on_set_parameters_callback(
            functools.partial(self.parameters_callback,
                              status_rate_params=status_rate_params,
                              sensor_rate_params=sensor_rate_params))
        
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
            self.declare_parameter(name, default_rates_hz.get(name, 1.0),
                ParameterDescriptor(description='Publish rate for robot status topics.',
                                    type=ParameterType.PARAMETER_DOUBLE,
                                    floating_point_range=[FloatingPointRange(
                                        from_value=0.0, to_value=1.0e9, step=0.0)],
                                    read_only=True))
        
        for name in sensor_rate_params:
            self.declare_parameter(name, default_rates_hz.get(name, 1.0),
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

        self.declare_parameter('has_eap_2', False,
            ParameterDescriptor(description='Set true if this robot features the Spot EAP2 payload.',
                                type=ParameterType.PARAMETER_BOOL,
                                read_only=True))

        self.declare_parameter('kinematic_model', 'none',
            ParameterDescriptor(description='The kinematic model for the Spot urdf, used to publish corresponding virtual joint states here',
                                type=ParameterType.PARAMETER_STRING,
                                additional_constraints="'none', 'body_assist', or 'mobile_manipulation'",
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
        
        self.declare_parameter('launch_pointcloud_service', False,
            ParameterDescriptor(description='Launch the robot pointcloud service instead of interfacing with the LiDAR directly',
                                type=ParameterType.PARAMETER_BOOL,
                                read_only=True))
        
        self.declare_parameter('obstacle_avoidance_padding', 0.10,
            ParameterDescriptor(description='Desired padding around the body to use when attempting to avoid obstacles. Described in meters',
                                type=ParameterType.PARAMETER_DOUBLE,
                                floating_point_range=[FloatingPointRange(from_value=0.0, to_value=0.5, step=0.0)],
                                read_only=False))

        self.declare_parameter('max_vel.x', 0.85,
            ParameterDescriptor(description="Maximum velocity of the robot in the x-direction. Units of m/s",
                                type=ParameterType.PARAMETER_DOUBLE,
                                floating_point_range=[FloatingPointRange(from_value=0.15, to_value=2.0, step=0.0)],
                                read_only=False))
        
        self.declare_parameter('max_vel.y', 0.5,
            ParameterDescriptor(description="Maximum velocity of the robot in the y-direction. Units of m/s",
                                type=ParameterType.PARAMETER_DOUBLE,
                                floating_point_range=[FloatingPointRange(from_value=0.15, to_value=2.0, step=0.0)],
                                read_only=False))
        
        self.declare_parameter('max_vel.theta', 1.0,
            ParameterDescriptor(description="Maximum rotational velocity of the robot. Units of rad/s",
                                type=ParameterType.PARAMETER_DOUBLE,
                                floating_point_range=[FloatingPointRange(from_value=0.20, to_value=1.5, step=0.0)],
                                read_only=False))

        self.declare_parameter('data_capture_mode', False,
            ParameterDescriptor(description='Whether we are in the mode to capture manipulation action-server goals.',
                                type=ParameterType.PARAMETER_BOOL,
                                read_only=True))

    def __del__(self):
        if self.status_timer is not None:
            self.status_timer.destroy()

        if self.spot_wrapper is None:
            return

        if not self.spot_wrapper.is_connected:
            return

        if self.spot_wrapper.is_standing:
            print('Spot sitting down...')
            is_sitting, message = self.spot_wrapper.sit()
        
            if not is_sitting:
                print('Not shutting down because Spot cannot sit here! ' + message)
                return

        print('Shutting down ROS driver for Spot')
        self.spot_wrapper.release()

    def RobotStateCB(self) -> None:
        """Callback for when the Spot Wrapper gets new robot state data."""
        self.spot_wrapper.updateState()

        state = self.spot_wrapper.robot_state
        if state is None:
            return

        odom_mode = self.get_parameter('odom_mode').value
        data_capture_mode = self.get_parameter('data_capture_mode').value
        
        # joint states #
        joint_state = JointStatesToMsg(state.kinematic_state, self.spot_wrapper)

        # Add in the virtual joints #
        kinematic_model = self.get_parameter('kinematic_model').value
        virtual_joint_state = GetVirtualJointValues(state.kinematic_state, kinematic_model, data_capture_mode)
        joint_state.name.extend(virtual_joint_state.name)
        joint_state.position.extend(virtual_joint_state.position)
        joint_state.velocity.extend(virtual_joint_state.velocity)
        joint_state.effort.extend(virtual_joint_state.effort)
        
        # TF #
        tf_msg = GetTFFromState(state.kinematic_state, self.spot_wrapper, kinematic_model)

        self.joint_state_pub.publish(joint_state)
        if len(tf_msg.transforms) > 0:
            self.tf_broadcaster.sendTransform(tf_msg.transforms)

        # Odom #
        odom_msg = GetOdomFromState(state.kinematic_state, self.spot_wrapper, odom_mode == 'vision')
        self.odom_pub.publish(odom_msg)
        
        # Odom Twist #
        twist_odom_msg = TwistStamped()
        twist_odom_msg.header = odom_msg.header
        twist_odom_msg.twist = odom_msg.twist.twist
        self.odom_twist_pub.publish(twist_odom_msg)
        
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

    def LeaseCB(self) -> None:
        """Callback for when the Spot Wrapper gets new lease data."""
        self.spot_wrapper._lease_manager.updateLeaseInfo()
        
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

    def PointCloudCB(self) -> None:
        """Callback for when the Spot Wrapper gets new pointcloud data."""
        self.spot_wrapper.updatePointCloud()

        if self.spot_wrapper.point_clouds is None:
            return
        
        for idx, pointcloud in enumerate(self.spot_wrapper.point_clouds):
            if self.point_cloud_pubs[idx].get_subscription_count() > 0:
                pointcloud_msg = PointCloudToMsg(pointcloud, self.spot_wrapper)
                if pointcloud_msg is not None:
                    self.point_cloud_pubs[idx].publish(pointcloud_msg)
        
    def handle_claim(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the claim service"""
        res.success = self.spot_wrapper.claim()
        return res

    def handle_release(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the release service"""
        res.success, res.message = self.spot_wrapper.release()
        return res
    
    def handle_force_claim(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the force_claim service"""
        res.success = self.spot_wrapper.claim(force=True)
        return res

    def handle_stop(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the stop service"""
        res.success, res.message = self.spot_wrapper.stop()
        return res

    def handle_self_right(self, _, res: Trigger.Response) -> Trigger.Response:
        """ROS service handler for the self-right service"""
        res.success, res.message = self.spot_wrapper.self_right()
        return res

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
            res.message = 'Powered on Spot robot ' + self.spot_wrapper.robot_id.nickname
        return res

    def handle_safe_power_off(self, _, res:Trigger.Response) -> Trigger.Response:
        """ROS service handler for the safe-power-off service"""
        res.success, res.message = self.spot_wrapper.power_off()
        return res
    
    def handle_estop_freeze(self, _, res:Trigger.Response) -> Trigger.Response:
        """ROS service handler to freeze the robot in place and prevent further movement"""
        res.success, res.message = self.spot_wrapper.freeze()
        return res
    
    def handle_estop_unfreeze(self, _, res:Trigger.Response) -> Trigger.Response:
        """ROS service handler to unfreeze the robot and allow new commands to be executed"""
        self.spot_wrapper.unfreeze()
        res.success = True
        res.message = "Robot can now accept new commands"
        return res

    def handle_estop_hard(self, _, res:Trigger.Response) -> Trigger.Response:
        """ROS service handler to hard-eStop the robot.  The robot will immediately cut power to the motors"""
        res.success, res.message = self.spot_wrapper._lease_manager.assertEStop(True)
        return res

    def handle_estop_soft(self, _, res:Trigger.Response) -> Trigger.Response:
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting
        power to the motors """
        res.success, res.message = self.spot_wrapper._lease_manager.assertEStop(False)
        return res

    def handle_estop_disengage(self, _, res:Trigger.Response) -> Trigger.Response:
        """ROS service handler to disengage the eStop on the robot."""
        res.success, res.message = self.spot_wrapper._lease_manager.disengageEStop()
        return res

    def handle_clear_behavior_fault(self, req, res: ClearBehaviorFault.Response) -> ClearBehaviorFault.Response:
        """ROS service handler for clearing behavior faults"""
        res.success, res.message = self.spot_wrapper.clear_behavior_fault(req.id)
        return res
    
    def handle_register_payload(self, req: RegisterPayload.Request, res: RegisterPayload.Response) -> RegisterPayload.Response:
        frame_names = {
            "body": MountFrameName.MOUNT_FRAME_BODY_PAYLOAD,
            "gripper": MountFrameName.MOUNT_FRAME_GRIPPER_PAYLOAD,
            "wrist": MountFrameName.MOUNT_FRAME_WR1
        }
        
        payload = Payload(
            GUID=req.guid,
            name=req.name,
            description=req.description,
            serial_number=req.serial_number,
            label_prefix=req.label_prefix,
            is_noncompute_payload=req.is_noncompute_payload,
            version=MsgToSoftwareVersion(req.version),
            mount_frame_name=frame_names[req.mount_frame],
            liveness_timeout_secs=req.liveness_timeout_secs,
            ipv4_address=req.ipv4_address,
            link_speed=req.link_speed,
            mass_volume_properties=MsgToPayloadMassVolumeProperties(req.mass_volume_properties)
        )

        res.success, res.message = self.spot_wrapper._lease_manager.register_payload(payload, req.secret)
        return res
    
    def handle_toggle_payload(self, req: TogglePayload.Request, res: TogglePayload.Response) -> TogglePayload.Response:
        if (len(req.guid) == 0) and (len(req.name) > 0):
            identifier = req.name
            use_name = True
        else:
            identifier = req.guid
            use_name = False

        res.success, res.message = self.spot_wrapper._lease_manager.toggle_payload(identifier, req.secret, req.attached, use_name)
        return res

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
        
    def handle_new_goal(self, goal_request):
        if not self.spot_wrapper._lease_manager.robot.is_powered_on():
            self.get_logger().warn('Cannot accept movement goal: robot is not powered on')
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT

    def handle_walk_to_accepted(self, goal_handle: ServerGoalHandle) -> None:
        if self.walk_to_active:
            self.get_logger().info('Received new goal during execution. Preempting previous goal')
            self.updated_walk_to_goal = goal_handle
        else:
            self.get_logger().info('Received new WalkTo goal')
            goal_handle.execute()

    def handle_walk_to_canceled(self, cancel_request) -> CancelResponse:
        self.get_logger().info('Canceling WalkTo goal')
        self.spot_wrapper.stop()
        return CancelResponse.ACCEPT

    def handle_walk_to(self, goal_handle: ServerGoalHandle) -> WalkTo.Result:
        req: WalkTo.Goal = goal_handle.request
        resp = WalkTo.Result()
        self.walk_to_active = True
        self.updated_walk_to_goal = None

        feedback_strings = {
            "STATUS" : [
                "STATUS_UNKNOWN: STATUS_UNKNOWN should never be used. If used, an internal error has happened.",
                "STATUS_STOPPED: The robot has stopped. Either the robot has reached the end of the trajectory or it "
                "                believes that it cannot reach the desired position. Robot may start to move again if "
                "                a blocked path clears.",
                "STATUS_IN_PROGRESS: The robot is actively following the requested trajectory.",
                "STATUS_STOPPING: The robot is nearing the end of the requested trajectory and is doing final positioning.",
            ],
            "BODY_STATUS" : [
                "BODY_STATUS_UNKNOWN: STATUS_UNKNOWN should never be used. If used, an internal error has happened.",
                "BODY_STATUS_MOVING: The robot body is not settled at the goal.",
                "BODY_STATUS_SETTLED: The robot is at the goal and the body has stopped moving."
            ],
            "GOAL_STATUS" : [
                "FINAL_GOAL_STATUS_UNKNOWN: FINAL_GOAL_STATUS_UNKNOWN should never be used. If used, an internal error has happened.",
                "FINAL_GOAL_STATUS_IN_PROGRESS: Robot is not stopped or stopping.",
                "FINAL_GOAL_STATUS_ACHIEVABLE: Final position was achievable.",
                "FINAL_GOAL_STATUS_BLOCKED: Final position was not achievable."
            ]
        }

        # Check to see if the pose is very old - if it is then update to now time
        msg_time = Time.from_msg(req.target_pose.header.stamp)
        seconds, nanoseconds = msg_time.seconds_nanoseconds()
        if seconds or nanoseconds:
            time_offset: rclpy.duration.Duration = self.get_clock().now() - msg_time
            if time_offset > rclpy.duration.Duration(seconds=10):
                self._logger.warn("Received WalkTo goal with a very old timestamp. Updating with current timestamp")
                req.target_pose.header.stamp = self.get_clock().now().to_msg()

        # Transform the target frame into the odom frame
        try:
            target_pose_in_odom = self.tf_buffer.transform(req.target_pose, "odom", rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().info(f"Unable to transform WalkTo target pose from {req.target_pose.header.frame_id} to the odom frame, aborting action: {e}")
            goal_handle.abort()
            resp.success = False
            resp.message = f"Unable to transform WalkTo target pose from {req.target_pose.header.frame_id} to the odom frame, aborting action: {e}"
            return resp
        
        # Convert the ROS types to the corresponding protobuf types
        target_pose_se2 = MsgToSE2Pose(target_pose_in_odom.pose).to_proto()
        max_vel = MsgToSE2Vel(req.max_vel).to_proto()

        self.get_logger().info(f"Moving robot to position ({target_pose_se2.position.x, target_pose_se2.position.y}) in the odom frame")

        def abort(message: str):
            self.get_logger().error(message)
            self.spot_wrapper.stop()
            self.walk_to_active = False
            self.updated_walk_to_goal = None
            goal_handle.abort()
            resp.success = False
            resp.message = message
            return resp

        # Make the command and make sure it was valid
        try:
            command_accepted, message, command_id = self.spot_wrapper.walk_to(target_pose_in_odom=target_pose_se2, max_vel=max_vel, max_duration=req.maximum_movement_time)
            if not command_accepted:
                return abort(f"Unable to command robot to move. Reason: {message}")
            else:
                self.get_logger().info(f"Started robot motion. Message: {message}")
        except Exception as e:
            return abort(f"Execption thrown in WalkTo action robot command execution: {e}")

        update_rate = self.create_rate(10.0)
        while rclpy.ok():
            # Check to see if the motion has been canceled
            if goal_handle.is_cancel_requested:
                resp.success = False
                resp.message = "Goal canceled"
                self.walk_to_active = False
                goal_handle.canceled()
                return resp
            
            # Check to see if we've received a new goal
            if self.updated_walk_to_goal is not None:
                resp.success = False
                resp.message = "Preempted by new goal"
                goal_handle.abort()
                self.updated_walk_to_goal.execute()
                return resp

            # Check to see if we've concluded
            try:
                command_feedback = self.spot_wrapper._lease_manager.robot_command_feedback(command_id)
                trajectory_feedback = command_feedback.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback
            except Exception as e:
                return abort(f"Execption thrown while getting command feedback: {e}")
            try:
                if trajectory_feedback.body_movement_status == WalkTo.Feedback.BODY_STATUS_SETTLED:
                    self.get_logger().info("WalkTo action completed successfully")
                    goal_handle.succeed()
                    resp.success = True
                    self.walk_to_active = False
                    resp.message = "WalkTo action completed successfully"
                    return resp
                elif trajectory_feedback.status == WalkTo.Feedback.STATUS_UNKNOWN:
                    return abort("Robot is in an unknown state. Aborting motion")
                elif trajectory_feedback.final_goal_status == WalkTo.Feedback.FINAL_GOAL_STATUS_BLOCKED:
                    return abort("Final goal is not achievable, aborting motion")
                else:
                    feedback_msg = WalkTo.Feedback()
                    feedback_msg.status_enum = trajectory_feedback.status
                    feedback_msg.status_string = feedback_strings["STATUS"][feedback_msg.status_enum]
                    feedback_msg.body_status_enum = trajectory_feedback.body_movement_status
                    feedback_msg.body_status_string = feedback_strings["BODY_STATUS"][feedback_msg.body_status_enum]
                    feedback_msg.final_goal_status_enum = trajectory_feedback.final_goal_status
                    feedback_msg.final_goal_status_string = feedback_strings["GOAL_STATUS"][feedback_msg.final_goal_status_enum]
                    goal_handle.publish_feedback(feedback_msg)
                    update_rate.sleep()
            except Exception as e:
                return abort(f"Exception thrown while checking feedback: {e}. Aborting motion")

    def cmdVelCallback(self, data: Twist) -> None:
        """Callback for cmd_vel command"""
        self.spot_wrapper.velocity_cmd(data.linear.x, data.linear.y, data.angular.z, cmd_duration=0.2)

    def bodyPoseCallback(self, data: Pose) -> None:
        """Callback for cmd_vel command"""
        try:
            rotation = MsgToQuaternion(data.orientation).to_proto()
            self.spot_wrapper.set_mobility_params(body_height_offset=data.position.z, footprint_R_body=to_euler_zxy(rotation))
            self.spot_wrapper.stand()
        except Exception as e:
            self._logger.error(f"Error setting body pose: {e}")

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

    def parameters_callback(self, params, status_rate_params, sensor_rate_params) -> SetParametersResult:
        if (self.spot_wrapper is None):
            return SetParametersResult(successful=True)

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
            elif p.name == 'obstacle_avoidance_padding':
                self.spot_wrapper._mobility_params.obstacle_avoidance_padding = p.value
            elif p.name == "max_vel.x":
                self.get_logger().info(f'Setting max-x to {p.value}')
                self.spot_wrapper._mobility_params.vel_limit.max_vel.linear.x = p.value
                self.spot_wrapper._max_cmd_x = p.value
            elif p.name == "max_vel.y":
                self.get_logger().info(f'Setting max-y to {p.value}')
                self.spot_wrapper._mobility_params.vel_limit.max_vel.linear.y = p.value
                self.spot_wrapper._max_cmd_y = p.value
            elif p.name == "max_vel.theta":
                self.get_logger().info(f'Setting max-theta to {p.value}')
                self.spot_wrapper._mobility_params.vel_limit.max_vel.angular = p.value
                self.spot_wrapper._max_cmd_rot = p.value
        
        return SetParametersResult(successful=True)

    def connect(self, lease_manager: SpotLeaseManager) -> bool:
        """
            Main function for the SpotROS class.
            Gets config from ROS and initializes the wrapper.
            Holds lease from wrapper and updates all async tasks at the ROS rate
        """

        ### --- Setup publishers and callbacks as requested --- ###
        self.get_logger().info("Setting sensor callbacks")
        callbacks = {}

        # Optional arguments
        has_cam_payload = self.get_parameter('has_cam_payload').value
        has_eap_2 = self.get_parameter('has_eap_2').value

        # Connect to the robot
        self.spot_wrapper = SpotBodyWrapper(self.get_logger(), self.get_parameter('hostname').value, has_eap_2, has_cam_payload)

        # Apply mobility parameters
        self.spot_wrapper.set_mobility_params(
            obstacle_avoidance_padding=self.get_parameter('obstacle_avoidance_padding').value,
            speed_limit=geometry_pb2.SE2Velocity(
                linear=geometry_pb2.Vec2(
                    x = self.get_parameter('max_vel.x').value,
                    y = self.get_parameter('max_vel.y').value),
                angular=self.get_parameter('max_vel.theta').value
            )    
        )

        # Dictionary of all param values in the 'rates' namespace
        status_rates_dict = {name: value.value for name, value in self.get_parameters_by_prefix('rates.status').items() }
        sensor_rates_dict = {name: value.value for name, value in self.get_parameters_by_prefix('rates.sensors').items() }

        # Pointcloud
        if self.get_parameter('launch_pointcloud_service').value:
            point_cloud_sources = {}

            if has_eap_2:
                self._logger.info("Launching EAP2 pointcloud service")
                point_cloud_sources['velodyne-point-cloud'] = 'velodyne_points'
                self.point_cloud_pubs = [self.create_publisher(PointCloud2, f"~/{topic}", 10) for (_, topic) in point_cloud_sources.items()]
                self.pointcloud_timer = self.create_timer(1/sensor_rates_dict["point_cloud"], self.PointCloudCB)
            else:
                self._logger.warn("Pointcloud service requested but robot does not have EAP2")
                sensor_rates_dict.pop('point_cloud')
        else:
            sensor_rates_dict.pop('point_cloud')

        self.get_logger().info(f"Status Rates: {status_rates_dict}")
        self.get_logger().info(f"Sensor Rates: {sensor_rates_dict}")

        # Setup timers for the state tasks
        self.state_timer = self.create_timer(1/status_rates_dict['robot_state'], self.RobotStateCB , callback_group=MutuallyExclusiveCallbackGroup())
        self.idle_timer  = self.create_timer(1/status_rates_dict['feedback'   ], self.publishStatus, callback_group=MutuallyExclusiveCallbackGroup())
        self.lease_timer = self.create_timer(1/status_rates_dict['lease'      ], self.LeaseCB      , callback_group=MutuallyExclusiveCallbackGroup())

        # Verify connection
        if self.spot_wrapper.connect(lease_manager):
            self.get_logger().info(f'Connected to Spot {self.spot_wrapper.robot_id.nickname}...')
        else:
            self.get_logger().fatal('Failed to launch ROS driver!')
            return False

        # Startup routine per parameter configuration
        if self.get_parameter('auto_claim').value:
            if self.spot_wrapper.claim():
                self.get_logger().info(f'Claimed lease on Spot robot {self.spot_wrapper.id.nickname}...')
                if self.get_parameter('auto_power_on').value:
                    self.get_logger().info('Spot powered on...')
                    if self.spot_wrapper.power_on():
                        if self.get_parameter('auto_stand').value:
                            self.get_logger().info('Spot standing up...')
                            pyTime.sleep(1.0)
                            self.spot_wrapper.stand()

        ## --- Status Publishers --- ##
        
        # QoS to use for latched publishers
        latched_qos = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        self.odom_pub            = self.create_publisher(Odometry                  , '~/odometry'              , 10)
        self.feet_pub            = self.create_publisher(FootStateArray            , '~/status/feet'           , 10)
        self.wifi_pub            = self.create_publisher(WiFiState                 , '~/status/wifi'           , qos_profile=latched_qos)
        self.lease_pub           = self.create_publisher(LeaseArray                , '~/status/leases'         , 1)
        self.power_pub           = self.create_publisher(PowerState                , '~/status/power_state'    , 1)
        self.estop_pub           = self.create_publisher(EStopStateArray           , '~/status/estop'          , 1)
        self.battery_pub         = self.create_publisher(BatteryStateArray         , '~/status/battery_states' , 1)
        self.dock_state_pub      = self.create_publisher(DockState                 , '~/status/dock_state'     , qos_profile=latched_qos)
        self.odom_twist_pub      = self.create_publisher(TwistStamped              , '~/odometry/twist'        , 1)
        self.joint_state_pub     = self.create_publisher(JointState                , '~/joint_states'          , 1)
        self.system_faults_pub   = self.create_publisher(SystemFaultState          , '~/status/system_faults'  , 10)
        self.behavior_faults_pub = self.create_publisher(BehaviorFaultState        , '~/status/behavior_faults', 10)
        self.mobility_params_pub = self.create_publisher(MobilityParams            , '~/status/mobility_params', 1)
        self.feedback_pub        = self.create_publisher(Feedback                  , '~/status/feedback'       , qos_profile=latched_qos)


        ## --- Controller Subscriptions --- ##

        body_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Twist, '~/cmd_vel'  , self.cmdVelCallback  , qos_profile_sensor_data, callback_group=body_callback_group)
        self.create_subscription(Pose , '~/body_pose', self.bodyPoseCallback, qos_profile_sensor_data, callback_group=body_callback_group)
 
        ## --- Services --- ##

        # Use callback group to prevent any services from attempting to execute simultaneously
        srv_group = MutuallyExclusiveCallbackGroup()

        # Status change services
        self.create_service(Trigger, "~/claim"      , self.handle_claim,          callback_group=srv_group)
        self.create_service(Trigger, "~/release"    , self.handle_release,        callback_group=srv_group)
        self.create_service(Trigger, "~/force_claim", self.handle_force_claim,    callback_group=srv_group)
        self.create_service(Trigger, "~/stop"       , self.handle_stop,           callback_group=srv_group)
        self.create_service(Trigger, "~/self_right" , self.handle_self_right,     callback_group=srv_group)
        self.create_service(Trigger, "~/sit"        , self.handle_sit,            callback_group=srv_group)
        self.create_service(Trigger, "~/stand"      , self.handle_stand,          callback_group=srv_group)
        self.create_service(Trigger, "~/power_on"   , self.handle_power_on,       callback_group=srv_group)
        self.create_service(Trigger, "~/power_off"  , self.handle_safe_power_off, callback_group=srv_group)

        # EStop services (no exclusive callback group so estop can interrupt other actions)
        self.create_service(Trigger, "~/estop/freeze"  , self.handle_estop_freeze)
        self.create_service(Trigger, "~/estop/unfreeze", self.handle_estop_unfreeze)
        self.create_service(Trigger, "~/estop/hard"    , self.handle_estop_hard)
        self.create_service(Trigger, "~/estop/gentle"  , self.handle_estop_soft)
        self.create_service(Trigger, "~/estop/release" , self.handle_estop_disengage, callback_group=srv_group)

        # Configuration services
        self.create_service(SetBool           , "~/stair_mode"          , self.handle_stair_mode,           callback_group=srv_group)
        self.create_service(SetLocomotion     , "~/locomotion_mode"     , self.handle_locomotion_mode,      callback_group=srv_group)
        self.create_service(SetVelocity       , "~/max_velocity"        , self.handle_max_vel,              callback_group=srv_group)
        self.create_service(ClearBehaviorFault, "~/clear_behavior_fault", self.handle_clear_behavior_fault, callback_group=srv_group)
        self.create_service(TogglePayload     , "~/toggle_payload"      , self.handle_toggle_payload,       callback_group=srv_group)
        self.create_service(RegisterPayload   , "~/register_payload"    , self.handle_register_payload,     callback_group=srv_group)

        # Status request services
        self.create_service(ListGraph, "~/list_graph", self.handle_list_graph, callback_group=srv_group)

        # Docking services
        self.create_service(Dock,    '~/dock',   self.handle_dock,   callback_group=srv_group)
        self.create_service(Trigger, '~/undock', self.handle_undock, callback_group=srv_group)

        # Gesture Services
        self.create_service(GestureSequence, "~/gesture_sequence",  self.handle_gesture_sequence,     callback_group=srv_group)

        ## --- Action Servers --- ##

        self._navigate_to_server = ActionServer(
            self,
            NavigateTo,
            '~/navigate_to',
            execute_callback=self.handle_navigate_to,
            callback_group=srv_group
        )
        
        self.walk_to_active = False
        self.updated_walk_to_goal: ServerGoalHandle = None
        self._walk_to_server = ActionServer(
            self,
            WalkTo,
            '~/walk_to',
            execute_callback=self.handle_walk_to,
            goal_callback=self.handle_new_goal,
            handle_accepted_callback=self.handle_walk_to_accepted,
            cancel_callback=self.handle_walk_to_canceled,
            callback_group=srv_group
        )

        # Publish initial dock state. Wait for first response
        while self.spot_wrapper.get_docking_state().status == DockState.DOCK_STATUS_UNKNOWN:
            pass
        self.update_dock_state()

        self.get_logger().info('Spot driver startup complete.')
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

    def publishStatus(self):
        if self.spot_wrapper is None:
            return

        if not self.spot_wrapper.is_connected:
            return
        
        self.spot_wrapper.update_idle_state()

        # publish robot feedback state
        feedback_msg = Feedback()
        feedback_msg.standing = self.spot_wrapper.is_standing
        feedback_msg.sitting  = self.spot_wrapper.is_sitting
        feedback_msg.moving = self.spot_wrapper.is_moving
        feedback_msg.docked = self.spot_wrapper.is_docked
        robot_id = self.spot_wrapper.robot_id
        if robot_id:
            feedback_msg.serial_number = robot_id.serial_number
            feedback_msg.species = robot_id.species
            feedback_msg.version = robot_id.version
            feedback_msg.nickname = robot_id.nickname
            feedback_msg.computer_serial_number = robot_id.computer_serial_number
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
    
    def handle_gesture_sequence(self, request: GestureSequence.Request, response: GestureSequence.Response) -> GestureSequence.Response:
        """ROS service handler for spot to execute gesture sequence"""
        if self.spot_wrapper is None:
            response.success = False
            response.message = "Spot wrapper is not initialized"
            return response

        # Dispatch table with *callables*, not immediate results
        MODE_HANDLERS = {
            "gesture_sequence": lambda req: self.spot_wrapper.perform_gesture(req.gesture_sequence),
            "sassy_confused": lambda req: self.spot_wrapper.sassy_confused(),
            "no_nod": lambda req: self.spot_wrapper.no_nod(),
            "water_shakeoff": lambda req: self.spot_wrapper.water_shakeoff(),
            "serious_stance": lambda req: self.spot_wrapper.serious_stance(),
            # Add more modes here...
        }

        handler = MODE_HANDLERS.get(request.gesture_mode)
        if handler is None:
            response.success = False
            response.message = f"Unknown mode: {request.gesture_mode}"
            return response

        success, message = handler(request)
        pyTime.sleep(1.0)
        response.success = success
        response.message = message
        return response
