from ast import Param
from email import message
import rclpy
from rclpy.node import Node
import rclpy.action
import rclpy.callback_groups

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
from spot_msgs.msg import TrajectoryAction, TrajectoryResult, TrajectoryFeedback
from spot_msgs.srv import ListGraph, ListGraphResponse
from spot_msgs.srv import SetLocomotion, SetLocomotionResponse
from spot_msgs.srv import ClearBehaviorFault, ClearBehaviorFaultResponse
from spot_msgs.srv import SetVelocity, SetVelocityResponse

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
                self.tf_pub.publish(tf_msg)

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
            metrics_msg.header.stamp = rospy.Time(local_time.seconds, local_time.nanos)

            for metric in metrics.metrics:
                if metric.label == "distance":
                    metrics_msg.distance = metric.float_value
                if metric.label == "gait cycles":
                    metrics_msg.gait_cycles = metric.int_value
                if metric.label == "time moving":
                    metrics_msg.time_moving = rospy.Time(metric.duration.seconds, metric.duration.nanos)
                if metric.label == "electric power":
                    metrics_msg.electric_power = rospy.Time(metric.duration.seconds, metric.duration.nanos)

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

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])
            self.populate_camera_static_transforms(data[2])
            self.populate_camera_static_transforms(data[3])

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

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])
            self.populate_camera_static_transforms(data[2])
            self.populate_camera_static_transforms(data[3])

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

            self.populate_camera_static_transforms(data[0])
            self.populate_camera_static_transforms(data[1])

    def handle_claim(self, req):
        """ROS service handler for the claim service"""
        resp = self.spot_wrapper.claim()
        return TriggerResponse(resp[0], resp[1])

    def handle_release(self, req):
        """ROS service handler for the release service"""
        resp = self.spot_wrapper.release()
        return TriggerResponse(resp[0], resp[1])

    def handle_stop(self, req):
        """ROS service handler for the stop service"""
        resp = self.spot_wrapper.stop()
        return TriggerResponse(resp[0], resp[1])

    def handle_self_right(self, req):
        """ROS service handler for the self-right service"""
        resp = self.spot_wrapper.self_right()
        return TriggerResponse(resp[0], resp[1])

    def handle_sit(self, req):
        """ROS service handler for the sit service"""
        resp = self.spot_wrapper.sit()
        return TriggerResponse(resp[0], resp[1])

    def handle_stand(self, req):
        """ROS service handler for the stand service"""
        resp = self.spot_wrapper.stand()
        return TriggerResponse(resp[0], resp[1])

    def handle_power_on(self, req):
        """ROS service handler for the power-on service"""
        resp = self.spot_wrapper.power_on()
        return TriggerResponse(resp[0], resp[1])

    def handle_safe_power_off(self, req):
        """ROS service handler for the safe-power-off service"""
        resp = self.spot_wrapper.safe_power_off()
        return TriggerResponse(resp[0], resp[1])

    def handle_estop_hard(self, req):
        """ROS service handler to hard-eStop the robot.  The robot will immediately cut power to the motors"""
        resp = self.spot_wrapper.assertEStop(True)
        return TriggerResponse(resp[0], resp[1])

    def handle_estop_soft(self, req):
        """ROS service handler to soft-eStop the robot.  The robot will try to settle on the ground before cutting
        power to the motors """
        resp = self.spot_wrapper.assertEStop(False)
        return TriggerResponse(resp[0], resp[1])

    def handle_estop_disengage(self, req):
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
            self.spot_wrapper.set_mobility_params( mobility_params )
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

    def handle_max_vel(self, req):
        """
        Handle a max_velocity service call. This will modify the mobility params to have a limit on the maximum
        velocity that the robot can move during motion commmands. This affects trajectory commands and velocity
        commands

        Args:
            req: SetVelocityRequest containing requested maximum velocity

        Returns: SetVelocityResponse
        """
        try:
            mobility_params = self.spot_wrapper.get_mobility_params()
            mobility_params.vel_limit.CopyFrom(SE2VelocityLimit(max_vel=math_helpers.SE2Velocity(req.velocity_limit.linear.x,
                                                                                                 req.velocity_limit.linear.y,
                                                                                                 req.velocity_limit.angular.z).to_proto()))
            self.spot_wrapper.set_mobility_params(mobility_params)
            return SetVelocityResponse(True, 'Success')
        except Exception as e:
            return SetVelocityResponse(False, 'Error:{}'.format(e))

    def handle_trajectory(self, req):
        """ROS actionserver execution handler to handle receiving a request to move to a location"""
        if req.target_pose.header.frame_id != 'body':
            self.trajectory_server.set_aborted(TrajectoryResult(False, 'frame_id of target_pose must be \'body\''))
            return
        if req.duration.data.to_sec() <= 0:
            self.trajectory_server.set_aborted(TrajectoryResult(False, 'duration must be larger than 0'))
            return

        cmd_duration = rospy.Duration(req.duration.data.secs, req.duration.data.nsecs)
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

        def timeout_cb(trajectory_server, _):
            trajectory_server.publish_feedback(TrajectoryFeedback("Failed to reach goal, timed out"))
            trajectory_server.set_aborted(TrajectoryResult(False, "Failed to reach goal, timed out"))

        # Abort the actionserver if cmd_duration is exceeded - the driver stops but does not provide feedback to
        # indicate this so we monitor it ourselves
        cmd_timeout = rospy.Timer(cmd_duration, functools.partial(timeout_cb, self.trajectory_server), oneshot=True)

        # The trajectory command is non-blocking but we need to keep this function up in order to interrupt if a
        # preempt is requested and to return success if/when the robot reaches the goal. Also check the is_active to
        # monitor whether the timeout_cb has already aborted the command
        rate = self.create_rate(10)
        while not rospy.is_shutdown() and not self.trajectory_server.is_preempt_requested() and not self.spot_wrapper.at_goal and self.trajectory_server.is_active():
            if self.spot_wrapper.near_goal:
                if self.spot_wrapper._last_trajectory_command_precise:
                    self.trajectory_server.publish_feedback(TrajectoryFeedback("Near goal, performing final adjustments"))
                else:
                    self.trajectory_server.publish_feedback(TrajectoryFeedback("Near goal"))
            else:
                self.trajectory_server.publish_feedback(TrajectoryFeedback("Moving to goal"))
            rate.sleep()

        # If still active after exiting the loop, the command did not time out
        if self.trajectory_server.is_active():
            cmd_timeout.shutdown()
            if self.trajectory_server.is_preempt_requested():
                self.trajectory_server.publish_feedback(TrajectoryFeedback("Preempted"))
                self.trajectory_server.set_preempted()
                self.spot_wrapper.stop()

            if self.spot_wrapper.at_goal:
                self.trajectory_server.publish_feedback(TrajectoryFeedback("Reached goal"))
                self.trajectory_server.set_succeeded(TrajectoryResult(resp[0], resp[1]))
            else:
                self.trajectory_server.publish_feedback(TrajectoryFeedback("Failed to reach goal"))
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
        while not rospy.is_shutdown() and self.run_navigate_to:
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

    def populate_camera_static_transforms(self, image_data: bosdyn.api.image_pb2.ImageResponse):
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
        excluded_frames = ['odom', 'vision', "body"]
        all_tfs_from_data = image_data.shot.transforms_snapshot.child_to_parent_edge_map
        existing_transforms = [(transform.header.frame_id, transform.child_frame_id) for transform in self.camera_static_transforms]
        filtered_tfs = [x for x in all_tfs_from_data
            if x.value.parent_frame_name not in excluded_frames
            and (x.value.parent_frame_name, x.key) not in existing_transforms]

        for tf in filtered_tfs:
            transform = image_data.shot.transforms_snapshot.child_to_parent_edge_map.get(frame_name)
            local_time = self.spot_wrapper.robotToLocalTime(image_data.shot.acquisition_time)
            tf_time = rospy.Time(local_time.seconds, local_time.nanos)
            static_tf = populateTransformStamped(tf_time, transform.parent_frame_name, frame_name,
                                                 transform.parent_tform_child)
            self.camera_static_transforms.append(static_tf)
            self.camera_static_transform_broadcaster.sendTransform(self.camera_static_transforms)

    def shutdown(self, force: bool) -> None:
        is_sitting, message = self.spot_wrapper.sit()[0:1]
        
        if not is_sitting and not force:
            self.get_logger.error('Not shutting down because Spot cannot sit here! ' + message)
            return

        self.get_logger.info('Shutting down ROS driver for Spot')
        self.spot_wrapper.disconnect()

    def parameters_callback(self, params):
        for p in params:
            if p.name == 'odom_mode':
                allowed = set('odom','vision')
                if p.value.string_value not in allowed:
                    return SetParametersResult(
                        successful=False,
                        reason="Parameter 'odom_mode' must take value 'odom' or 'vision'.")

        return SetParametersResult(successful=True)

    def main(self):
        """Main function for the SpotROS class.  Gets config from ROS and initializes the wrapper.  Holds lease from wrapper and updates all async tasks at the ROS rate"""

        ''' ROS Parameter Descriptors '''
        username_descriptor = ParameterDescriptor('Spot computer username.',
                                                  type=ParameterType.PARAMETER_STRING,
                                                  value='default_value',
                                                  read_only=True)

        password_descriptor = ParameterDescriptor('Spot computer password.',
                                                  type=ParameterType.PARAMETER_STRING,
                                                  value='default_value',
                                                  read_only=True)

        hostname_descriptor = ParameterDescriptor('Spot computer hostname.',
                                                  type=ParameterType.PARAMETER_STRING,
                                                  value='default_value',
                                                  read_only=True)

        rates_descriptor = ParameterDescriptor('TODO',
                                               value={},
                                               read_only=True)

        motion_deadzone_descriptor = ParameterDescriptor('TODO',
                                                         type=ParameterType.PARAMETER_INTEGER,
                                                         value=0.05,
                                                         floating_point_range=FloatingPointRange(
                                                            from_value=0, to_value=1e9, step=0),
                                                         read_only=True)

        estop_timeout_descriptor = ParameterDescriptor('The E-Stop engages if we lose connection for this long.',
                                                       type=ParameterType.PARAMETER_INTEGER,
                                                       value=9.0,
                                                       floating_point_range=FloatingPointRange(
                                                          from_value=0, to_value=1e9, step=0),
                                                       read_only=True)
                                                       
        self.add_on_set_parameters_callback(self.parameters_callback)

        ''' ROS Parameters '''
        self.username = self.declare_parameter('username', username_descriptor).value
        self.password = self.declare_parameter('password', password_descriptor).value
        self.hostname = self.declare_parameter('hostname', hostname_descriptor).value
        self.rates = self.declare_parameter('rates', rates_descriptor).value
        self.deadzone = self.declare_parameter('deadzone', motion_deadzone_descriptor).value
        self.estop_timeout = self.declare_parameter('estop_timeout', estop_timeout_descriptor).value

        self.camera_static_transform_broadcaster = tf2_ros.StaticTransformBroadcaster()
        # Static transform broadcaster is super simple and just a latched publisher. Every time we add a new static
        # transform we must republish all static transforms from this source, otherwise the tree will be incomplete.
        # We keep a list of all the static transforms we already have so they can be republished, and so we can check
        # which ones we already have
        self.camera_static_transforms = []

        # Spot has 2 types of odometries: 'odom' and 'vision'
        # The former one is kinematic odometry and the second one is a combined odometry of vision and kinematics
        # These params enables to change which odometry frame is a parent of body frame and to change tf names of each odometry frames.
        self.mode_parent_odom_tf = rospy.get_param('~mode_parent_odom_tf', 'odom') # 'vision' or 'odom'
        self.tf_name_kinematic_odom = rospy.get_param('~tf_name_kinematic_odom', 'odom')

        odom_mode_descriptor = ParameterDescriptor("Selects pure kinematic odometry or fused vision and kinematic odometry.",
                                                   type=ParameterType.PARAMETER_STRING,
                                                   value='odom',
                                                   additional_constraints="'odom' or 'vision'",
                                                   read_only=True)
        self.odom_mode = self.declare_parameter('odom_mode', odom_mode_descriptor).value

        self.tf_name_raw_kinematic = 'odom'
        self.tf_name_vision_odom = rospy.get_param('~tf_name_vision_odom', 'vision')
        self.tf_name_raw_vision = 'vision'

        self.logger = logging.getLogger('rosout')

        rospy.loginfo("Starting ROS driver for Spot")
        self.callbacks = {}
        """Dictionary listing what callback to use for what data task"""
        self.callbacks["robot_state"] = self.RobotStateCB
        self.callbacks["metrics"] = self.MetricsCB
        self.callbacks["lease"] = self.LeaseCB
        self.callbacks["front_image"] = self.FrontImageCB
        self.callbacks["side_image"] = self.SideImageCB
        self.callbacks["rear_image"] = self.RearImageCB
        self.spot_wrapper = SpotWrapper(self.username, self.password, self.hostname, self.logger, self.estop_timeout, self.rates, self.callbacks)

        if self.spot_wrapper.is_valid:
            # Images #
            self.back_image_pub = self.create_publisher(Image, 'camera/back/image')
            self.frontleft_image_pub = self.create_publisher(Image, 'camera/frontleft/image')
            self.frontright_image_pub = self.create_publisher(Image, 'camera/frontright/image')
            self.left_image_pub = self.create_publisher(Image, 'camera/left/image')
            self.right_image_pub = self.create_publisher(Image, 'camera/right/image')
            # Depth #
            self.back_depth_pub = self.create_publisher(Image, 'depth/back/image')
            self.frontleft_depth_pub = self.create_publisher(Image, 'depth/frontleft/image')
            self.frontright_depth_pub = self.create_publisher(Image, 'depth/frontright/image')
            self.left_depth_pub = self.create_publisher(Image, 'depth/left/image')
            self.right_depth_pub = self.create_publisher(Image, 'depth/right/image')

            # Image Camera Info #
            self.back_image_info_pub = self.create_publisher(CameraInfo, 'camera/back/camera_info',)
            self.frontleft_image_info_pub = self.create_publisher(CameraInfo, 'camera/frontleft/camera_info')
            self.frontright_image_info_pub = self.create_publisher(CameraInfo, 'camera/frontright/camera_info')
            self.left_image_info_pub = self.create_publisher(CameraInfo, 'camera/left/camera_info')
            self.right_image_info_pub = self.create_publisher(CameraInfo, 'camera/right/camera_info')

            # Depth Camera Info #
            self.back_depth_info_pub = self.create_publisher(CameraInfo, 'depth/back/camera_info')
            self.frontleft_depth_info_pub = self.create_publisher(CameraInfo, 'depth/frontleft/camera_info')
            self.frontright_depth_info_pub = self.create_publisher(CameraInfo, 'depth/frontright/camera_info')
            self.left_depth_info_pub = self.create_publisher(CameraInfo, 'depth/left/camera_info')
            self.right_depth_info_pub = self.create_publisher(CameraInfo, 'depth/right/camera_info')

            # Status Publishers #
            self.joint_state_pub = self.create_publisher(JointState, 'joint_states')
            """Defining a TF publisher manually because of conflicts between Python3 and tf"""
            self.tf_pub = self.create_publisher(TFMessage, 'tf')
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
            self.feedback_pub = self.create_publisher(Feedback, 'status/feedback')
            self.mobility_params_pub = self.create_publisher(MobilityParams, 'status/mobility_params')

            self.create_subscription(Twist, 'cmd_vel', self.cmdVelCallback)
            self.create_subscription(Pose, 'body_pose', self.bodyPoseCallback)

            self.create_service(Trigger, "claim", self.handle_claim)
            self.create_service(Trigger, "release", self.handle_release)
            self.create_service(Trigger, "stop", self.handle_stop)
            self.create_service(Trigger, "self_right", self.handle_self_right)
            self.create_service(Trigger, "sit", self.handle_sit)
            self.create_service(Trigger, "stand", self.handle_stand)
            self.create_service(Trigger, "power_on", self.handle_power_on)
            self.create_service(Trigger, "power_off", self.handle_safe_power_off)

            self.create_service(Trigger, "estop/hard", self.handle_estop_hard)
            self.create_service(Trigger, "estop/gentle", self.handle_estop_soft)
            self.create_service(Trigger, "estop/release", self.handle_estop_disengage)

            self.create_service(SetBool, "stair_mode", self.handle_stair_mode)
            self.create_service(SetLocomotion, "locomotion_mode", self.handle_locomotion_mode)
            self.create_service(SetVelocity, "max_velocity", self.handle_max_vel)
            self.create_service(ClearBehaviorFault, "clear_behavior_fault", self.handle_clear_behavior_fault)

            self.create_service(ListGraph, "list_graph", self.handle_list_graph)

            self._action_server = rclpy.action.ActionServer(
                    self,
                    NavigateToAction,
                    'navigate_to',
                    execute_callback=self.handle_navigate_to,
                    callback_group=rclpy.callback_groups.ReentrantCallbackGroup()

            
            self.navigate_as.start()

            self._action_server = rclpy.action.ActionServer(
                    self,
                    TrajectoryAction,
                    'trajectory',
                    execute_callback=self.handle_trajectory,
                    callback_group=rclpy.callback_groups.ReentrantCallbackGroup())

            self.trajectory_server.start()

            rospy.on_shutdown(self.shutdown)

            self.auto_claim = rospy.get_param('~auto_claim', False)
            self.auto_power_on = rospy.get_param('~auto_power_on', False)
            self.auto_stand = rospy.get_param('~auto_stand', False)

            if self.auto_claim:
                self.spot_wrapper.claim()
                if self.auto_power_on:
                    self.spot_wrapper.power_on()
                    if self.auto_stand:
                        self.spot_wrapper.stand()

            rate = self.create_rate(50)
            while rclpy.ok():
                self.spot_wrapper.updateTasks()
                feedback_msg = Feedback()
                feedback_msg.standing = self.spot_wrapper.is_standing
                feedback_msg.sitting = self.spot_wrapper.is_sitting
                feedback_msg.moving = self.spot_wrapper.is_moving
                id = self.spot_wrapper.id
                try:
                    feedback_msg.serial_number = id.serial_number
                    feedback_msg.species = id.species
                    feedback_msg.version = id.version
                    feedback_msg.nickname = id.nickname
                    feedback_msg.computer_serial_number = id.computer_serial_number
                except:
                    pass
                self.feedback_pub.publish(feedback_msg)
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
                    rospy.logerr('Error:{}'.format(e))
                    pass
                self.mobility_params_pub.publish(mobility_params_msg)
                rate.sleep()