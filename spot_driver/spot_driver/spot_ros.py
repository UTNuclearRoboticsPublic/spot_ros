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

import rclpy
from rclpy.node import Node
import rclpy.action
import rclpy.callback_groups
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from rcl_interfaces.msg import FloatingPointRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import SetParametersResult

from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist, Pose
from nav_msgs.msg import Odometry

from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api import geometry_pb2, trajectory_pb2
from bosdyn.api.geometry_pb2 import Quaternion, SE2VelocityLimit
from bosdyn.client import math_helpers
import bosdyn.geometry

import functools
import tf2_ros

from spot_msgs.msg import Metrics
from spot_msgs.msg import LeaseArray, LeaseResource
from spot_msgs.msg import FootState, FootStateArray
from spot_msgs.msg import EStopState, EStopStateArray
from spot_msgs.msg import WiFiState
from spot_msgs.msg import PowerState
from spot_msgs.msg import BehaviorFault, BehaviorFaultState
from spot_msgs.msg import SystemFault, SystemFaultState
from spot_msgs.msg import BatteryState, BatteryStateArray
from spot_msgs.msg import Feedback
from spot_msgs.msg import MobilityParams
from spot_msgs.msg import NavigateToAction, NavigateToResult, NavigateToFeedback
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal, TrajectoryResult, TrajectoryFeedback
from spot_msgs.srv import ListGraph, ListGraphResponse
from spot_msgs.srv import SetLocomotion, SetLocomotionResponse
from spot_msgs.srv import ClearBehaviorFault, ClearBehaviorFaultResponse
from spot_msgs.srv import SetVelocity, SetVelocityRequest, SetVelocityResponse

from .ros_helpers import *
from .spot_wrapper import SpotWrapper

import logging
import threading

class SpotROS(Node):
    """Parent class for using the wrapper.  Defines all callbacks and keeps the wrapper alive"""

    def __init__(self):
        super().__init__('spot_driver')

        self.spot_wrapper = None

    def RobotStateCB(self, results):
        """Callback for when the Spot Wrapper gets new robot state data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        state = self.spot_wrapper.robot_state

        if state:
            ## joint states ##
            joint_state = GetJointStatesFromState(state, self.spot_wrapper)
            self.joint_state_pub.publish(joint_state)

            ## TF ##
            tf_msg = GetTFFromState(state, self.spot_wrapper, self.mode_parent_odom_tf)
            if len(tf_msg.transforms) > 0:
                self.tf_broadcaster.sendTransform(tf_msg)

            # Odom Twist #
            twist_odom_msg = GetOdomTwistFromState(state, self.spot_wrapper)
            self.odom_twist_pub.publish(twist_odom_msg)

            # Odom #
            if self.mode_parent_odom_tf == 'vision':
                odom_msg = GetOdomFromState(state, self.spot_wrapper, use_vision=True)
            else:
                odom_msg = GetOdomFromState(state, self.spot_wrapper, use_vision=False)
            self.odom_pub.publish(odom_msg)

            # Feet #
            foot_array_msg = GetFeetFromState(state, self.spot_wrapper)
            self.feet_pub.publish(foot_array_msg)

            # EStop #
            estop_array_msg = GetEStopStateFromState(state, self.spot_wrapper)
            self.estop_pub.publish(estop_array_msg)

            # WIFI #
            wifi_msg = GetWifiFromState(state, self.spot_wrapper)
            self.wifi_pub.publish(wifi_msg)

            # Battery States #
            battery_states_array_msg = GetBatteryStatesFromState(state, self.spot_wrapper)
            self.battery_pub.publish(battery_states_array_msg)

            # Power State #
            power_state_msg = GetPowerStatesFromState(state, self.spot_wrapper)
            self.power_pub.publish(power_state_msg)

            # System Faults #
            system_fault_state_msg = GetSystemFaultsFromState(state, self.spot_wrapper)
            self.system_faults_pub.publish(system_fault_state_msg)

            # Behavior Faults #
            behavior_fault_state_msg = getBehaviorFaultsFromState(state, self.spot_wrapper)
            self.behavior_faults_pub.publish(behavior_fault_state_msg)

    def MetricsCB(self, results):
        """Callback for when the Spot Wrapper gets new metrics data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        metrics = self.spot_wrapper.metrics
        if metrics:
            metrics_msg = Metrics()
            local_time = self.spot_wrapper.robotToLocalTime(metrics.timestamp)
            metrics_msg.header.stamp = rclpy.time.Time(local_time.seconds, local_time.nanos)

            for metric in metrics.metrics:
                if metric.label == "distance":
                    metrics_msg.distance = metric.float_value
                elif metric.label == "gait cycles":
                    metrics_msg.gait_cycles = metric.int_value
                elif metric.label == "time moving":
                    metrics_msg.time_moving = rclpy.time.Time(metric.duration.seconds, metric.duration.nanos)
                elif metric.label == "electric power":
                    metrics_msg.electric_power = rclpy.time.Time(metric.duration.seconds, metric.duration.nanos)

            self.metrics_pub.publish(metrics_msg)

    def LeaseCB(self, results):
        """Callback for when the Spot Wrapper gets new lease data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """
        lease_array_msg = LeaseArray()
        lease_list = self.spot_wrapper.lease
        if lease_list:
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

    def FrontImageCB(self, results):
        """Callback for when the Spot Wrapper gets new front image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """

        # [front left image, front right image, front left depth, front right depth]
        data = self.spot_wrapper.front_images

        if data and len(data) == 4:
            # front left image
            image_msg, camera_info_msg = getImageMsg(data[0], self.spot_wrapper)
            self.frontleft_image_pub.publish(image_msg)
            self.frontleft_image_info_pub.publish(camera_info_msg)

            # front right image
            image_msg, camera_info_msg = getImageMsg(data[1], self.spot_wrapper)
            self.frontright_image_pub.publish(image_msg)
            self.frontright_image_info_pub.publish(camera_info_msg)

            # front left depth
            image_msg, camera_info_msg = getImageMsg(data[2], self.spot_wrapper)
            self.frontleft_depth_pub.publish(image_msg)
            self.frontleft_depth_info_pub.publish(camera_info_msg)

            # front right depth
            image_msg, camera_info_msg = getImageMsg(data[3], self.spot_wrapper)
            self.frontright_depth_pub.publish(image_msg)
            self.frontright_depth_info_pub.publish(camera_info_msg)

    def SideImageCB(self, results):
        """Callback for when the Spot Wrapper gets new side image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """

        # [left image, right image, left depth, right depth]
        data = self.spot_wrapper.side_images

        if data and len(data) == 4:
            # left image
            image_msg, camera_info_msg = getImageMsg(data[0], self.spot_wrapper)
            self.left_image_pub.publish(image_msg)
            self.left_image_info_pub.publish(camera_info_msg)

            # right image
            image_msg, camera_info_msg = getImageMsg(data[1], self.spot_wrapper)
            self.right_image_pub.publish(image_msg)
            self.right_image_info_pub.publish(camera_info_msg)

            # left depth
            image_msg, camera_info_msg = getImageMsg(data[2], self.spot_wrapper)
            self.left_depth_pub.publish(image_msg)
            self.left_depth_info_pub.publish(camera_info_msg)

            # right depth
            image_msg, camera_info_msg = getImageMsg(data[3], self.spot_wrapper)
            self.right_depth_pub.publish(image_msg)
            self.right_depth_info_pub.publish(camera_info_msg)

    def RearImageCB(self, results):
        """Callback for when the Spot Wrapper gets new rear image data.

        Args:
            results: FutureWrapper object of AsyncPeriodicQuery callback
        """

        # [image, depth]
        data = self.spot_wrapper.rear_images

        if data and len(data) == 2:
            # image
            image_msg, camera_info_msg = getImageMsg(data[0], self.spot_wrapper)
            self.back_image_pub.publish(image_msg)
            self.back_image_info_pub.publish(camera_info_msg)

            # depth
            image_msg, camera_info_msg = getImageMsg(data[1], self.spot_wrapper)
            self.back_depth_pub.publish(image_msg)
            self.back_depth_info_pub.publish(camera_info_msg)

    def handle_claim(self, _):
        """ROS service handler for the claim service"""
        resp = self.spot_wrapper.claim()
        return TriggerResponse(resp[0], resp[1])

    def handle_release(self, _):
        """ROS service handler for the release service"""
        resp = self.spot_wrapper.release()
        return TriggerResponse(resp[0], resp[1])

    def handle_stop(self, _):
        """ROS service handler for the stop service"""
        resp = self.spot_wrapper.stop()
        return TriggerResponse(resp[0], resp[1])

    def handle_self_right(self, _):
        """ROS service handler for the self-right service"""
        resp = self.spot_wrapper.self_right()
        return TriggerResponse(resp[0], resp[1])

    def handle_sit(self, _):
        """ROS service handler for the sit service"""
        resp = self.spot_wrapper.sit()
        return TriggerResponse(resp[0], resp[1])

    def handle_stand(self, _):
        """ROS service handler for the stand service"""
        resp = self.spot_wrapper.stand()
        return TriggerResponse(resp[0], resp[1])

    def handle_power_on(self, _):
        """ROS service handler for the power-on service"""
        resp = self.spot_wrapper.power_on()
        return TriggerResponse(resp[0], resp[1])

    def handle_safe_power_off(self, _):
        """ROS service handler for the safe-power-off service"""
        resp = self.spot_wrapper.safe_power_off()
        return TriggerResponse(resp[0], resp[1])

    def handle_estop_hard(self, _):
        """ROS service handler to hard-eStop the robot.  The robot will immediately cut power to the motors"""
        resp = self.spot_wrapper.assertEStop(True)
        return TriggerResponse(resp[0], resp[1])

    def handle_estop_soft(self, _):
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting
        power to the motors """
        resp = self.spot_wrapper.assertEStop(False)
        return TriggerResponse(resp[0], resp[1])

    def handle_estop_disengage(self, _):
        """ROS service handler to disengage the eStop on the robot."""
        resp = self.spot_wrapper.disengageEStop()
        return TriggerResponse(resp[0], resp[1])

    def handle_clear_behavior_fault(self, req):
        """ROS service handler for clearing behavior faults"""
        resp = self.spot_wrapper.clear_behavior_fault(req.id)
        return ClearBehaviorFaultResponse(resp[0], resp[1])

    def handle_stair_mode(self, req):
        """ROS service handler to set a stair mode to the robot."""
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.stair_hint = req.data
            self.spot_wrapper.set_mobility_params(mobility_params)
            return SetBoolResponse(True, 'Success')
        except Exception as e:
            return SetBoolResponse(False, 'Error:{}'.format(e))

    def handle_locomotion_mode(self, req):
        """ROS service handler to set locomotion mode"""
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.locomotion_hint = req.locomotion_mode
            self.spot_wrapper.set_mobility_params( mobility_params )
            return SetLocomotionResponse(True, 'Success')
        except Exception as e:
            return SetLocomotionResponse(False, 'Error:{}'.format(e))

    def handle_max_vel(self, req: SetVelocityRequest):
        """
        Handle a max_velocity service call. This will modify the mobility params to set a limit on the maximum
        velocity that the robot can move during motion commmands. This affects trajectory commands and velocity
        commands

        Args:
            req: SetVelocityRequest containing requested maximum velocity

        Returns: SetVelocityResponse
        """
        if (req.velocity_limit.linear.x == 0.0 or
            req.velocity_limit.linear.y == 0.0 or
            req.velocity_limit.linear.z == 0.0):
            return SetVelocityResponse(False, 'Cannot set a velocity limit of zero.')

        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.vel_limit.CopyFrom(
                SE2VelocityLimit(max_vel=math_helpers.SE2Velocity(req.velocity_limit.linear.x,
                                                                  req.velocity_limit.linear.y,
                                                                  req.velocity_limit.angular.z).to_proto()))
            self.spot_wrapper.set_mobility_params(mobility_params)
            return SetVelocityResponse(True, 'Success')
        except Exception as e:
            return SetVelocityResponse(False, 'Error:{}'.format(e))

    def handle_trajectory(self, req: TrajectoryGoal) -> None:
        """ROS actionserver execution handler to handle receiving a request to move to a location"""
        if req.target_pose.header.frame_id != 'body':
            self.trajectory_server.set_aborted(
                TrajectoryResult(False, 'frame_id of target_pose must be \'body\''))
            return
        if req.duration.data.to_sec() <= 0:
            self.trajectory_server.set_aborted(TrajectoryResult(False, 'duration must be larger than 0'))
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
                self.trajectory_server.set_preempted(TrajectoryFeedback(False, "Preempted"))
                self.spot_wrapper.stop()
                return
            elif self.spot_wrapper.at_goal:
                self.trajectory_server.set_succeeded(TrajectoryResult(resp[0], resp[1]))
                return
            elif self.spot_wrapper.near_goal:
                if self.spot_wrapper._last_trajectory_command_precise:
                    self.trajectory_server.publish_feedback(
                        TrajectoryFeedback("Near goal, performing precise adjustments"))
                else:
                    self.trajectory_server.publish_feedback(TrajectoryFeedback("Near goal"))
            else:
                self.trajectory_server.publish_feedback(TrajectoryFeedback("Moving to goal"))

            # check for timeout
            if (self.get_clock().now() - start_time > cmd_duration):
                # the action has timed out. abort.
                self.trajectory_server.set_aborted(
                    TrajectoryResult(False, "Failed to reach goal, timed out"))
                return

            rate.sleep()

        # We timed out
        self.trajectory_server.set_aborted(TrajectoryResult(False, "Failed to reach goal"))

    def cmdVelCallback(self, data):
        """Callback for cmd_vel command"""
        self.spot_wrapper.velocity_cmd(data.linear.x, data.linear.y, data.angular.z)

    def bodyPoseCallback(self, data):
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

    def handle_list_graph(self, upload_path):
        """ROS service handler for listing graph_nav waypoint_ids"""
        resp = self.spot_wrapper.list_graph(upload_path)
        return ListGraphResponse(resp)

    def handle_navigate_to_feedback(self):
        """Thread function to send navigate_to feedback"""
        rate = self.create_rate(10)
        while rclpy.ok() and self.run_navigate_to:
            localization_state = self.spot_wrapper._graph_nav_client.get_localization_state()
            if localization_state.localization.waypoint_id:
                self.navigate_as.publish_feedback(NavigateToFeedback(localization_state.localization.waypoint_id))
            rate.sleep()

    def handle_navigate_to(self, msg):
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
            self.navigate_as.set_succeeded(NavigateToResult(resp[0], resp[1]))
        else:
            self.navigate_as.set_aborted(NavigateToResult(resp[0], resp[1]))

    def populate_camera_static_transforms(self,
                                          image_data: bosdyn.api.image_pb2.ImageResponse,
                                          existing_transforms: list) -> list:
        """Check data received from one of the image tasks and use the transform snapshot to extract the camera frame
        transforms. These are the transforms from body->frontleft->frontleft_fisheye, for example. These transforms
        never change, but they may be calibrated slightly differently for each robot so we need to generate the
        transforms at runtime.

        Args:
            image_data: ImageResponse protobuf data from the wrapper
        """

        # We exclude the odometry frames from static transforms since they are not static. We can ignore the body
        # frame because it is a child of odom or vision depending on the mode_parent_odom_tf, and will be published
        # by the non-static transform publishing that is done by the state callback
        excluded_frames = ['odom', 'vision', 'body']
        all_tfs_from_data = image_data.shot.transforms_snapshot.child_to_parent_edge_map
        existing_transforms = [(transform.header.frame_id, transform.child_frame_id) for transform in existing_transforms]
        
        tfs_to_add = [x for x in all_tfs_from_data
            if x.value.parent_frame_name not in excluded_frames
            and (x.value.parent_frame_name, x.key) not in existing_transforms]

        # tf: FrameTreeSnapshot.ChildToParentEdgeMapEntry
        #    key: str
        #    value: FrameTreeSnapshot.ParentEdge
        #       parent_frame_name: str
        #       parent_tform_child: bosdyn.client.math_helpers.SE3Pose
        output = existing_transforms
        for tf in tfs_to_add:
            local_time = self.spot_wrapper.robotToLocalTime(image_data.shot.acquisition_time)
            tf_time = rclpy.time.Time(local_time.seconds, local_time.nanos)
            static_tf = populateTransformStamped(tf_time,
                                                 tf.value.parent_frame_name,
                                                 tf.key,
                                                 tf.value.parent_tform_child)
            output.append(static_tf)
        
        return output

    def __del__(self) -> None:
        is_sitting, message = self.spot_wrapper.sit()[0:1]
        
        if not is_sitting:
            self.get_logger().error('Not shutting down because Spot cannot sit here! ' + message)
            return

        self.get_logger().info('Shutting down ROS driver for Spot')
        self.spot_wrapper.disconnect()

    def parameters_callback(self, params, rates_names) -> SetParametersResult:
        for p in params:
            if p.name == 'odom_mode':
                allowed = set('odom','vision')
                if p.value.string_value not in allowed:
                    return SetParametersResult(
                        successful=False,
                        reason="Parameter 'odom_mode' must take value 'odom' or 'vision'.")
            elif p.name in rates_names:
                if p.value.float_value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="Parameter rates/" + p.name + " must be positive.")

        return SetParametersResult(successful=True)


    def main(self):
        """Main function for the SpotROS class.  Gets config from ROS and initializes the wrapper.  Holds lease from wrapper and updates all async tasks at the ROS rate"""

        ''' ROS Parameters '''
        rates_names = ['robot_state', 'metrics', 'lease', 'front_image', 'size_image', 'rear_image']                                    
        self.add_on_set_parameters_callback(
            functools.partial(self.parameters_callback, rates_names=rates_names))
        
        username = self.declare_parameter('username',
            ParameterDescriptor('Spot computer username.',
                                type=ParameterType.PARAMETER_STRING,
                                value='default_value',
                                read_only=True)).value

        password = self.declare_parameter('password',
            ParameterDescriptor('Spot computer password.',
                                type=ParameterType.PARAMETER_STRING,
                                value='default_value',
                                read_only=True)).value
        
        hostname = self.declare_parameter('hostname',
            ParameterDescriptor('Spot computer hostname.',
                                type=ParameterType.PARAMETER_STRING,
                                value='default_value',
                                read_only=True)).value
        
        estop_timeout = self.declare_parameter('estop_timeout',
            ParameterDescriptor('The E-Stop engages if we lose connection for this long.',
                                type=ParameterType.PARAMETER_INTEGER,
                                value=9.0,
                                floating_point_range=FloatingPointRange(
                                    from_value=0, to_value=1e9, step=0),
                                read_only=True)).value

        rates = {}
        for name in rates_names:
            rates[name] = self.declare_parameter('rates/'+name,
                ParameterDescriptor('Publish rate for robot state topics.',
                                    type=ParameterType.PARAMETER_DOUBLE,
                                    value=0.0,
                                    floating_point_range=FloatingPointRange(
                                        from_value=0, to_value=1e9, step=0),
                                    read_only=True)).value

        # Spot has 2 types of odometries: 'odom' and 'vision'
        # The former one is kinematic odometry and the second one is a combined odometry of vision and kinematics
        self.odom_mode = self.declare_parameter('odom_mode',
            ParameterDescriptor("Selects pure kinematic odometry or fused vision and kinematic odometry.",
                                type=ParameterType.PARAMETER_STRING,
                                value='odom',
                                additional_constraints="'odom' or 'vision'",
                                read_only=True)).value

        """Dictionary listing what callback to use for what data task"""
        callbacks = {}
        callbacks["robot_state"] = self.RobotStateCB
        callbacks["metrics"]     = self.MetricsCB
        callbacks["lease"]       = self.LeaseCB
        callbacks["front_image"] = self.FrontImageCB
        callbacks["side_image"]  = self.SideImageCB
        callbacks["rear_image"]  = self.RearImageCB

        # Connect to the robot
        self.spot_wrapper = SpotWrapper(username, password, hostname, logging.getLogger('rosout'), estop_timeout, rates, callbacks)

        if self.spot_wrapper.is_valid:
            self.get_logger().info("Starting ROS driver for Spot")
        else:
            self.get_logger().fatal('Failed to launch Spot driver!')
            exit(1)

        # Images
        self.back_image_pub = self.create_publisher(Image, 'camera/back/image')
        self.frontleft_image_pub = self.create_publisher(Image, 'camera/frontleft/image')
        self.frontright_image_pub = self.create_publisher(Image, 'camera/frontright/image')
        self.left_image_pub = self.create_publisher(Image, 'camera/left/image')
        self.right_image_pub = self.create_publisher(Image, 'camera/right/image')
        # Depth
        self.back_depth_pub = self.create_publisher(Image, 'depth/back/image')
        self.frontleft_depth_pub = self.create_publisher(Image, 'depth/frontleft/image')
        self.frontright_depth_pub = self.create_publisher(Image, 'depth/frontright/image')
        self.left_depth_pub = self.create_publisher(Image, 'depth/left/image')
        self.right_depth_pub = self.create_publisher(Image, 'depth/right/image')
        # Image Camera Info
        self.back_image_info_pub = self.create_publisher(CameraInfo, 'camera/back/camera_info',)
        self.frontleft_image_info_pub = self.create_publisher(CameraInfo, 'camera/frontleft/camera_info')
        self.frontright_image_info_pub = self.create_publisher(CameraInfo, 'camera/frontright/camera_info')
        self.left_image_info_pub = self.create_publisher(CameraInfo, 'camera/left/camera_info')
        self.right_image_info_pub = self.create_publisher(CameraInfo, 'camera/right/camera_info')
        # Depth Camera Info
        self.back_depth_info_pub = self.create_publisher(CameraInfo, 'depth/back/camera_info')
        self.frontleft_depth_info_pub = self.create_publisher(CameraInfo, 'depth/frontleft/camera_info')
        self.frontright_depth_info_pub = self.create_publisher(CameraInfo, 'depth/frontright/camera_info')
        self.left_depth_info_pub = self.create_publisher(CameraInfo, 'depth/left/camera_info')
        self.right_depth_info_pub = self.create_publisher(CameraInfo, 'depth/right/camera_info')

        # Status Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states')
        self.metrics_pub = self.create_publisher(Metrics, 'status/metrics')
        self.lease_pub = self.create_publisher(LeaseArray, 'status/leases')
        self.odom_twist_pub = self.create_publisher(TwistWithCovarianceStamped, 'odometry/twist')
        self.odom_pub = self.create_publisher(Odometry, 'odometry')
        self.feet_pub = self.create_publisher(FootStateArray, 'status/feet')
        self.estop_pub = self.create_publisher(EStopStateArray, 'status/estop')
        self.wifi_pub = self.create_publisher(WiFiState, 'status/wifi')
        self.power_pub = self.create_publisher(PowerState, 'status/power_state')
        self.battery_pub = self.create_publisher(BatteryStateArray, 'status/battery_states')
        self.behavior_faults_pub = self.create_publisher(BehaviorFaultState, 'status/behavior_faults')
        self.system_faults_pub = self.create_publisher(SystemFaultState, 'status/system_faults')
        mobility_params_pub = self.create_publisher(MobilityParams, 'status/mobility_params')
        feedback_pub = self.create_publisher(Feedback, 'status/feedback',
            qos_profile=QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                   history=QoSHistoryPolicy.KEEP_LAST,
                                   depth=1))

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.create_subscription(Twist, 'cmd_vel', self.cmdVelCallback)
        self.create_subscription(Pose, 'body_pose', self.bodyPoseCallback)

        srv_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, "claim", self.handle_claim, callback_group=srv_group)
        self.create_service(Trigger, "release", self.handle_release, callback_group=srv_group)
        self.create_service(Trigger, "stop", self.handle_stop, callback_group=srv_group)
        self.create_service(Trigger, "self_right", self.handle_self_right, callback_group=srv_group)
        self.create_service(Trigger, "sit", self.handle_sit, callback_group=srv_group)
        self.create_service(Trigger, "stand", self.handle_stand, callback_group=srv_group)
        self.create_service(Trigger, "power_on", self.handle_power_on, callback_group=srv_group)
        self.create_service(Trigger, "power_off", self.handle_safe_power_off, callback_group=srv_group)

        self.create_service(Trigger, "estop/hard", self.handle_estop_hard, callback_group=srv_group)
        self.create_service(Trigger, "estop/gentle", self.handle_estop_soft, callback_group=srv_group)
        self.create_service(Trigger, "estop/release", self.handle_estop_disengage, callback_group=srv_group)

        self.create_service(SetBool, "stair_mode", self.handle_stair_mode, callback_group=srv_group)
        self.create_service(SetLocomotion, "locomotion_mode", self.handle_locomotion_mode, callback_group=srv_group)
        self.create_service(SetVelocity, "max_velocity", self.handle_max_vel, callback_group=srv_group)
        self.create_service(ClearBehaviorFault, "clear_behavior_fault", self.handle_clear_behavior_fault, callback_group=srv_group)

        self.create_service(ListGraph, "list_graph", self.handle_list_graph, callback_group=srv_group)

        nav_to_as = rclpy.action.ActionServer(
                self,
                NavigateToAction,
                'navigate_to',
                execute_callback=self.handle_navigate_to,
                callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        
        nav_to_as.start()

        trajectory_as = rclpy.action.ActionServer(
                self,
                TrajectoryAction,
                'trajectory',
                execute_callback=self.handle_trajectory,
                callback_group=rclpy.callback_groups.ReentrantCallbackGroup())

        trajectory_as.start()

        # populate the static transforms for the various robot cameras
        
        def populate_static_transforms() -> tf2_ros.StaticTransformBroadcaster:
            while not (self.spot_wrapper.front_images and len(self.spot_wrapper.front_images) == 4):
                pass
            while not (self.spot_wrapper.side_images and len(self.spot_wrapper.side_images) == 4):
                pass
            while not (self.spot_wrapper.rear_images and len(self.spot_wrapper.rear_images) == 2):
                pass

            static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
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

            static_tf_broadcaster.sendTransform(static_tfs)
            return static_tf_broadcaster

        _ = populate_static_transforms()

        self.auto_claim = self.declare_parameter('auto_claim',
            ParameterDescriptor('Automatically claim ownership of the robot on connection.',
                                type=ParameterType.PARAMETER_BOOL,
                                value=False,
                                read_only=True)).value

        self.auto_power_on = self.declare_parameter('auto_power_on',
            ParameterDescriptor('Automatically power on the robot on connection.',
                                type=ParameterType.PARAMETER_BOOL,
                                value=False,
                                read_only=True)).value

        self.auto_stand = self.declare_parameter('auto_stand',
            ParameterDescriptor('Automatically stand up the robot on connection.',
                                type=ParameterType.PARAMETER_BOOL,
                                value=False,
                                read_only=True)).value

        if self.auto_claim:
            self.spot_wrapper.claim()
            if self.auto_power_on:
                self.spot_wrapper.power_on()
                if self.auto_stand:
                    self.spot_wrapper.stand()        

        rate = self.create_rate(50)
        while rclpy.ok():
            # call all periodic tasks
            self.spot_wrapper.updateTasks()

            # publish robot feedback state
            feedback_msg = Feedback()
            feedback_msg.standing = self.spot_wrapper.is_standing
            feedback_msg.sitting = self.spot_wrapper.is_sitting
            feedback_msg.moving = self.spot_wrapper.is_moving
            id = self.spot_wrapper.id
            if id:
                feedback_msg.serial_number = id.serial_number
                feedback_msg.species = id.species
                feedback_msg.version = id.version
                feedback_msg.nickname = id.nickname
                feedback_msg.computer_serial_number = id.computer_serial_number
            feedback_pub.publish(feedback_msg)

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
            mobility_params_pub.publish(mobility_params_msg)

            rate.sleep()