############################################################################################
#      Title     : spot_wrapper.py
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

from typing import Text, Tuple
from .async_queries import *

from bosdyn.api import (image_pb2, header_pb2, geometry_pb2, trajectory_pb2, 
                        arm_command_pb2, gripper_command_pb2, synchronized_command_pb2)
from bosdyn.api.docking import docking_pb2
from bosdyn.api.spot import robot_command_pb2
from bosdyn.geometry import EulerZXY

from bosdyn.client import create_standard_sdk, ResponseError, RpcError, power
from bosdyn.client.auth import AuthResponseError
from bosdyn.client.async_tasks import AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.image import ImageClient, build_image_request
from bosdyn.client.lease import ResourceAlreadyClaimedError, InvalidResourceError, NotAuthoritativeServiceError, LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.spot_cam.audio import AudioClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_id import RobotIdClient
import bosdyn.client.util
from bosdyn.util import seconds_to_duration

from google.protobuf.timestamp_pb2 import Timestamp as PB2Timestamp
from google.protobuf.duration_pb2 import Duration as PB2Duration
from google.protobuf.message import Message as PB2Message

class SpotWrapper():
    """Generic wrapper class to encompass release 1.1.4 API features as well as maintaining leases automatically"""
    def __init__(self, has_cam_payload: bool = False):
        self._is_connected = False
        self._robot = None
        self._lease = None
        self._has_cam_payload = has_cam_payload

        self._mobility_params = RobotCommandBuilder.mobility_params()
        self._is_standing = False
        self._is_sitting = True
        self._is_moving = False
        self._last_docking_command = None
        self._last_stand_command = None
        self._last_sit_command = None
        self._last_trajectory_command = None
        self._last_trajectory_command_precise = None
        self._last_velocity_command_time = None

    def connect(self, logger, hostname, rates = {}, callbacks = {}) -> bool:
        front_image_sources = {'frontleft_fisheye_image', 'frontright_fisheye_image', 'frontleft_depth', 'frontright_depth'}
        side_image_sources = {'left_fisheye_image', 'right_fisheye_image', 'left_depth', 'right_depth'}
        rear_image_sources = {'back_fisheye_image', 'back_depth'}
        hand_image_sources = {'hand_image', 'hand_depth', 'hand_color_image', 'hand_depth_in_hand_color_frame'}
        self._logger = logger

        front_image_requests = []
        for source in front_image_sources:
            front_image_requests.append(build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW))

        side_image_requests = []
        for source in side_image_sources:
            side_image_requests.append(build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW))

        rear_image_requests = []
        for source in rear_image_sources:
            rear_image_requests.append(build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW))

        hand_image_requests = []
        for source in hand_image_sources:
            hand_image_requests.append(build_image_request(source, image_format=image_pb2.Image.FORMAT_RAW))

        try:
            self._sdk = bosdyn.client.create_standard_sdk('ros_spot')
        except IOError as err:
            logger.error('Error creating SDK object ' + Text(err))
            return False

        self._robot = self._sdk.create_robot(hostname)

        try:
            bosdyn.client.util.authenticate(self._robot)
        except RpcError as err:
            logger.error('Failed to communicate with robot {}: {}'.format(hostname, err.error_message))
            return False
        except AuthResponseError as err:
            logger.error('Authentication failed. ' + err.error_message)
            return False

        self._robot.start_time_sync()

        # Spot service clients
        try:
            self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
            self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
            self._power_client = self._robot.ensure_client(PowerClient.default_service_name)
            self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)
            self._image_client = self._robot.ensure_client(ImageClient.default_service_name)
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._docking_client = self._robot.ensure_client(DockingClient.default_service_name) 
        except Exception as e:
            logger.error('Unable to create client service: ' + Text(e))
            return False

        if self._has_cam_payload:
            try:
                self._audio_client = self._robot.ensure_client(AudioClient.default_service_name)
            except Exception as e:
                logger.error('Unable to create client service: ' + Text(e))
                return False

        # Async Tasks
        self._robot_state_task = AsyncRobotState(self._robot_state_client, logger, rates.get("status.robot_state", 1.0), callbacks.get("robot_state", lambda:None))
        self._lease_task = AsyncLease(self._lease_client, logger, rates.get("status.lease", 1.0), callbacks.get("lease", lambda:None))
        self._front_image_task = AsyncImageService(self._image_client, logger, rates.get("sensors.front_image", 1.0), callbacks.get("front_image", lambda:None), front_image_requests)
        self._side_image_task = AsyncImageService(self._image_client, logger, rates.get("sensors.side_image", 1.0), callbacks.get("side_image", lambda:None), side_image_requests)
        self._rear_image_task = AsyncImageService(self._image_client, logger, rates.get("sensors.rear_image", 1.0), callbacks.get("rear_image", lambda:None), rear_image_requests)
        self._hand_image_task = AsyncImageService(self._image_client, logger, rates.get("sensors.hand_image", 1.0), callbacks.get("hand_image", lambda:None), hand_image_requests)
        self._idle_task = AsyncIdle(self._robot_command_client, logger, 10.0, self)

        self._estop_endpoint = None

        self._async_status_tasks = AsyncTasks([self._robot_state_task,
                                               self._lease_task,
                                               self._idle_task
                                              ])

        self._async_sensor_tasks = AsyncTasks([self._front_image_task,
                                               self._side_image_task,
                                               self._rear_image_task,
                                               self._hand_image_task
                                               ])

        self._is_connected = True
        return True

    @property
    def logger(self):
        """Return this wrapper's logger"""
        return self._logger

    @property
    def is_connected(self) -> bool:
        """Return boolean indicating if the wrapper initialized successfully"""
        return self._is_connected

    @property
    def id(self):
        """Return robot's ID"""
        if not self._is_connected:
            return None
            
        return self._robot.get_id()

    @property
    def robot_state(self):
        """Return latest proto from the _robot_state_task"""
        return self._robot_state_task.proto

    @property
    def lease(self):
        """Return latest proto from the _lease_task"""
        return self._lease_task.proto

    @property
    def front_images(self):
        """Return latest proto from the _front_image_task"""
        return self._front_image_task.proto

    @property
    def side_images(self):
        """Return latest proto from the _side_image_task"""
        return self._side_image_task.proto

    @property
    def rear_images(self):
        """Return latest proto from the _rear_image_task"""
        return self._rear_image_task.proto

    @property
    def hand_images(self):
        """Return latest proto from the _hand_image_task"""
        return self._hand_image_task.proto

    @property
    def is_standing(self) -> bool:
        """Return boolean of standing state"""
        return self._is_standing

    @property
    def is_sitting(self) -> bool:
        """Return boolean of standing state"""
        return self._is_sitting

    @property
    def is_moving(self) -> bool:
        """Return boolean of walking state"""
        return self._is_moving

    @property
    def time_skew(self) -> PB2Duration:
        """Return the time skew between local and spot time"""
        return self._robot.time_sync.endpoint.clock_skew

    def _robot_command(self, command_proto: PB2Message,
                       end_time_secs: float =None) -> Tuple[bool, Text]:
        """Generic blocking function for sending commands to robots.

        Args:
            command_proto: robot_command_pb2 protobuf message to send to the robot.
                           Usually made with RobotCommandBuilder
            end_time_secs: (optional) Time-to-live for the command in seconds
        """
        try:
            id = self._robot_command_client.robot_command(lease=None, command=command_proto, end_time_secs=end_time_secs)
            return True, "Success", id
        except Exception as e:
            return False, Text(e), None

    def robotToLocalTime(self, timestamp: PB2Timestamp) -> PB2Timestamp:
        """Takes a timestamp and an estimated skew and return seconds and nano seconds

        Args:
            timestamp: google.protobuf.Timestamp
        Returns:
            google.protobuf.Timestamp
        """

        rtime = PB2Timestamp()
        rtime.seconds = timestamp.seconds - self.time_skew.seconds
        rtime.nanos = timestamp.nanos - self.time_skew.nanos
        if rtime.nanos < 0:
            rtime.nanos = rtime.nanos + int(1e9)
            rtime.seconds = rtime.seconds - 1
        elif rtime.nanos > int(1e9):
            rtime.nanos = rtime.nanos - int(1e9)
            rtime.seconds = rtime.seconds + 1

        return rtime

    def claim(self) -> Tuple[bool, Text]:
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        try:
            if not self.getLease():
                return False
            self.resetEStop()
        except (ResponseError, RpcError) as err:
            return False, err.error_message
        
        return True, 'Success'

    def updateStatusTasks(self) -> None:
        """Loop through the state, and lease periodic tasks and update their data if needed."""
        self._async_status_tasks.update()

    def updateSensorTasks(self) -> None:
        """Loop through the sensor query periodic tasks and update their data if needed."""
        self._async_sensor_tasks.update()

    def resetEStop(self) -> None:
        """Get keepalive for eStop"""
        self._estop_endpoint = EstopEndpoint(self._estop_client, 'ros', 9.0)
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def assertEStop(self, severe=True) -> bool:
        """Forces the robot into eStop state.

        Args:
            severe: Default True - If true, will cut motor power immediately.  If false, will try to settle the robot on the ground first
        """
        try:
            if severe:
                self._estop_endpoint.stop()
            else:
                self._estop_endpoint.settle_then_cut()
        except Exception:
            return False

        return True

    def releaseEStop(self) -> None:
        """Stop eStop keepalive"""
        if self._estop_keepalive:
            self._estop_keepalive.stop()
            self._estop_keepalive = None
            self._estop_endpoint = None

    def getLease(self) -> Tuple[bool, Text]:
        """Get a lease for the robot and keep the lease alive automatically."""
        try:
            self._lease = self._lease_client.acquire()
        except (ResourceAlreadyClaimedError, InvalidResourceError, NotAuthoritativeServiceError) as err:
            return False, err.error_message
        
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)
        return True, 'Success'

    def releaseLease(self) -> None:
        """Return the lease on the body."""
        if self._lease:
            self._lease_client.return_lease(self._lease)
            self._lease = None

    def release(self) -> bool:
        """Return the lease on the body and the eStop handle."""
        try:
            self.sit()
            self.releaseLease()
            self.releaseEStop()
        except Exception as err:
            return False, Text(err)

        return True, 'Success'

    def disconnect(self) -> None:
        """Release control of robot as gracefully as posssible."""
        if self._robot is None:
            return

        if self._robot.time_sync:
            self._robot.time_sync.stop()
        self.release()

    def stop(self) -> Tuple[bool, Text]:
        """Stop the robot's motion."""
        response = self._robot_command(RobotCommandBuilder.stop_command())
        return response[0], response[1]

    def self_right(self) -> Tuple[bool, Text]:
        """Have the robot self-right itself."""
        response = self._robot_command(RobotCommandBuilder.selfright_command())
        return response[0], response[1]

    def sit(self) -> Tuple[bool, Text]:
        """Stop the robot's motion and sit down if able."""
        self.arm_stow()
        response = self._robot_command(RobotCommandBuilder.synchro_sit_command())
        self._last_sit_command = response[2]
        return response[0], response[1]

    def stand(self, monitor_command=True) -> Tuple[bool, Text]:
        """If the e-stop is enabled, and the motor power is enabled, stand the robot up."""
        response = self._robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))
        if monitor_command:
            self._last_stand_command = response[2]
        return response[0], response[1]

    def dock(self, dock_id) -> Tuple[bool, Text]:
        """Dock the robot to the docking station with fiducial ID [dock_id]."""
        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            self.stand()
            # Dock the robot
            self.last_docking_command = dock_id
            blocking_dock_robot(self._robot, dock_id)
            self.last_docking_command = None
        except Exception as e:
            return False, Text(e)
        return True, 'Success'

    def undock(self, timeout: float = 20.0) -> Tuple[bool, Text]:
        """Power motors on and undock the robot from the station."""
        try:
            # Make sure we're powered on
            self._robot.power_on()

            # Undock the robot
            blocking_undock(self._robot, timeout)
        except Exception as e:
            return False, Text(e)
        return True, 'Success'

    def get_docking_state(self, **kwargs) -> docking_pb2.DockState:
        """Get docking state of robot."""
        state = self._docking_client.get_docking_state(**kwargs)
        return state

    def safe_power_off(self) -> Tuple[bool, Text]:
        """Stop the robot's motion and sit if possible.  Once sitting, disable motor power."""
        response = self._robot_command(RobotCommandBuilder.safe_power_off_command())
        return response[0], response[1]

    def power_on(self) ->  Tuple[bool, Text]:
        """Enable the motor power if e-stop is enabled."""
        try:
            power.power_on(self._power_client)
            return True, 'Success'
        except Exception as e:
            return False, Text(e)

    def set_mobility_params(self,
                            body_height: float = 0.0,
                            footprint_R_body: EulerZXY = EulerZXY(),
                            locomotion_hint: int = 1,
                            stair_hint: bool = False,
                            external_force_params: robot_command_pb2.BodyExternalForceParams = None) -> None:
        """Define body, locomotion, and stair parameters.

        Args:
            body_height: Body height in meters
            footprint_R_body: (EulerZXY) – The orientation of the body frame with respect to the footprint frame (gravity aligned framed with yaw computed from the stance feet)
            locomotion_hint: Locomotion hint
            stair_hint: Boolean to define stair motion
        """
        self._mobility_params = RobotCommandBuilder.mobility_params(body_height, footprint_R_body, locomotion_hint, stair_hint, external_force_params)

    def get_mobility_params(self) -> robot_command_pb2.MobilityParams:
        """Get mobility params
        """
        return self._mobility_params

    def velocity_cmd(self, v_x: float, v_y: float, v_rot: float, cmd_duration=0.1) -> None:
        """Send a velocity motion command to the robot.

        Args:
            v_x: Velocity in the X direction in meters per second
            v_y: Velocity in the Y direction in meters per second
            v_rot: Angular velocity around the Z axis in radians per second
            cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 100ms (assuming 10Hz command rate).
        """
        end_time=time.time() + cmd_duration
        self._robot_command(RobotCommandBuilder.synchro_velocity_command(
                            v_x=v_x, v_y=v_y, v_rot=v_rot, params=self._mobility_params),
                            end_time_secs=end_time)
        self._last_velocity_command_time = end_time

    def play_sound(self, name: Text, gain: float, block: bool) -> Tuple[bool, Text]:
        if not self._has_cam_payload:
            return False, 'This Spot has no audio capability.'

        if not name:
            return False, Text('Spot needs non-empty name for sound file')
        if gain <= 0.0:
            return False, Text('Audio play gain must be positive.')

        if block:
            response = self._audio_client.load_sound(name, gain)
        else:
            response = self._audio_client.load_sound_async(name, gain)

        success = response.error.code == header_pb2.CommonError.Code.CODE_OK
        return success, response.error.message

    def load_sound(self, name: Text, data: bytes) -> Tuple[bool, Text]:
        if not self._has_cam_payload:
            return False, 'This Spot has no audio capability.'

        if not name:
            return False, Text('Spot needs non-empty name for sound file')
        if not data:
            return False, Text('Spot needs non-empty data for sound file')

        response = self._audio_client.load_sound(name, data)
        success = response.error.code == header_pb2.CommonError.Code.CODE_OK
        return success, response.error.message

    def set_volume(self, percentage: float) -> Tuple[bool, Text]:
        if not self._has_cam_payload:
            return False, 'This Spot has no audio capability.'
        
        if percentage > 100.0 or percentage < 0.0:
            return False, Text('Could not set audio volume to invalid percentage ' + percentage)

        response = self._audio_client.set_volume(percentage)
        success = response.error.code == header_pb2.CommonError.Code.CODE_OK
        return success, Text(response.error.message)


    # ARM ######
    def arm_carry(self) -> Tuple[bool, Text]:
        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            self.stand()

            self._robot_command_client.robot_command(RobotCommandBuilder.arm_carry_command())
            return True, 'Success'
        except Exception as e: 
            return False, Text(e)

    def arm_stow(self) -> Tuple[bool, Text]:
        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            self.stand()

            self._robot_command_client.robot_command(RobotCommandBuilder.arm_stow_command())
            return True, 'Success'
        except Exception as e:
            return False, Text(e)

    def arm_unstow(self) -> Tuple[bool, Text]:
        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            self.stand()

            self._robot_command_client.robot_command(RobotCommandBuilder.arm_ready_command())
            return True, 'Success'
        except Exception as e:
            return False, Text(e)

    def gripper_close(self) -> Tuple[bool, Text]:
        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            self.stand()
            self.arm_unstow()

            self._robot_command_client.robot_command(RobotCommandBuilder.claw_gripper_close_command())
            return True, 'Success'
        except Exception as e:
            return False, Text(e)

    def gripper_open(self) -> Tuple[bool, Text]:
        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            self.stand()
            self.arm_unstow()

            self._robot_command_client.robot_command(RobotCommandBuilder.claw_gripper_open_command())
            return True, 'Success'
        except Exception as e:
            return False, Text(e)

    # def set_gripper_params(self, max_velocity, max_acceleration)
    #     gripper_command_pb2.ClawGripperCommand.Request(maximum_open_close_velocity = 5, maximum_open_close_acceleration = 20)

    def gripper_angle_open(self, gripper_ang: float) -> Tuple[bool, Text]:
        if gripper_ang > 90.0 or gripper_ang < 0.0: 
            return False, Text('Could not set gripper angle to invalid angle' + gripper_ang)

        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            self.stand()
            self.arm_unstow()

            # The open angle command does not take degrees but the limits
            # defined in the urdf, that is why we have to interpolate
            closed = 0.349066
            opened = -1.396263
            angle = gripper_ang / 90.0 * (opened - closed) + closed

            self._robot_command_client.robot_command(RobotCommandBuilder.claw_gripper_open_angle_command(angle))
            return True, 'Success'
        except Exception as e:
            return False, Text(e)

    # def hand_pose(self, pose_points) -> Tuple[bool, Text]:
    #     try:
    #         # Make sure we're powered on and standing
    #         self._robot.power_on()
    #         self.stand()


    #     except Exception as e:
    #         return False, Text(e)

    def force_trajectory(self, data) -> Tuple[bool, Text]:
        try:
            # Make sure we're powered on and standing
            self._robot.power_on()
            self.stand()
            self.arm_unstow()

            def create_wrench_from_msg(forces, torques):
                force = geometry_pb2.Vec3(x=forces[0], y=forces[1], z=forces[2])
                torque = geometry_pb2.Vec3(x=torques[0], y=torques[1], z=torques[2])
                return geometry_pb2.Wrench(force=force, torque=torque)

            # Duration in seconds.
            traj_duration = 5

            # first point on trajectory
            wrench0 = create_wrench_from_msg(data.forces_pt0, data.torques_pt0)
            t0 = seconds_to_duration(0)
            traj_point0 = trajectory_pb2.WrenchTrajectoryPoint(
                wrench=wrench0, time_since_reference=t0
            )

            # Second point on the trajectory
            wrench1 = create_wrench_from_msg(data.forces_pt1, data.torques_pt1)
            t1 = seconds_to_duration(traj_duration)
            traj_point1 = trajectory_pb2.WrenchTrajectoryPoint(
                wrench=wrench1, time_since_reference=t1
            )

            # Build the trajectory
            trajectory = trajectory_pb2.WrenchTrajectory(
                points=[traj_point0, traj_point1]
            )

            # Build the trajectory request, putting all axes into force mode
            arm_cartesian_command = arm_command_pb2.ArmCartesianCommand.Request(
                root_frame_name=ODOM_FRAME_NAME,
                wrench_trajectory_in_task=trajectory,
                x_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                y_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                z_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                rx_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                ry_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
                rz_axis=arm_command_pb2.ArmCartesianCommand.Request.AXIS_MODE_FORCE,
            )
            arm_command = arm_command_pb2.ArmCommand.Request(
                arm_cartesian_command=arm_cartesian_command
            )
            synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
                arm_command=arm_command
            )
            robot_command = robot_command_pb2.RobotCommand(
                synchronized_command=synchronized_command
            )

            # Send the request
            self._robot_command_client.robot_command(robot_command)
            self._logger.info("Force trajectory command sent")

            time.sleep(10.0)
            
            return True, 'Success'
        except Exception as e:
            return False, Text(e)

    # def force_virtual_trajectory(self, data) -> Tuple[bool, Text]:
    #     """Send a 

    #     Args:
    #         v_x: Velocity in the X direction in meters per second
    #         v_y: Velocity in the Y direction in meters per second
    #         v_rot: Angular velocity around the Z axis in radians per second
    #         cmd_duration: (optional) Time-to-live for the command in seconds.  Default is 100ms (assuming 10Hz command rate).
    #     """
    #     try:
    #         # Make sure we're powered on and standing
    #         self._robot.power_on()
    #         self.stand()
    #         self.arm_unstow()


    #     except Exception as e:
    #         return False, Text(e)

    # def make_arm_trajectory_command(self, arm_joint_trajectory) -> Tuple[bool, Text]:
    #     try:
    #         # Make sure we're powered on and standing
    #         self._robot.power_on()
    #         self.stand()

    #     except Exception as e:
    #         return False, Text(e)


    # def arm_joint_move(self, joint_targets) -> Tuple[bool, Text]:
    #     # All perspectives are given when looking at the robot from behind after the unstow service is called
    #     # Joint1: 0.0 arm points to the front. positive: turn left, negative: turn right)
    #     # RANGE: -3.14 -> 3.14
    #     # Joint2: 0.0 arm points to the front. positive: move down, negative move up
    #     # RANGE: 0.4 -> -3.13 (
    #     # Joint3: 0.0 arm straight. moves the arm down
    #     # RANGE: 0.0 -> 3.1415
    #     # Joint4: 0.0 middle position. negative: moves ccw, positive moves cw
    #     # RANGE: -2.79253 -> 2.79253
    #     # # Joint5: 0.0 gripper points to the front. positive moves the gripper down
    #     # RANGE: -1.8326 -> 1.8326
    #     # Joint6: 0.0 Gripper is not rolled, positive is ccw
    #     # RANGE: -2.87 -> 2.87
    #     # Values after unstow are: [0.0, -0.9, 1.8, 0.0, -0.9, 0.0]
    #     if abs(joint_targets[0]) > 3.14:
    #         msg = "Joint 1 has to be between -3.14 and 3.14"
    #         self._logger.warn(msg)
    #         return False, msg
    #     elif joint_targets[1] > 0.4 or joint_targets[1] < -3.13:
    #         msg = "Joint 2 has to be between -3.13 and 0.4"
    #         self._logger.warn(msg)
    #         return False, msg
    #     elif joint_targets[2] > 3.14 or joint_targets[2] < 0.0:
    #         msg = "Joint 3 has to be between 0.0 and 3.14"
    #         self._logger.warn(msg)
    #         return False, msg
    #     elif abs(joint_targets[3]) > 2.79253:
    #         msg = "Joint 4 has to be between -2.79253 and 2.79253"
    #         self._logger.warn(msg)
    #         return False, msg
    #     elif abs(joint_targets[4]) > 1.8326:
    #         msg = "Joint 5 has to be between -1.8326 and 1.8326"
    #         self._logger.warn(msg)
    #         return False, msg
    #     elif abs(joint_targets[5]) > 2.87:
    #         msg = "Joint 6 has to be between -2.87 and 2.87"
    #         self._logger.warn(msg)
    #         return False, msg
    #     try:
    #         # Make sure we're powered on and standing
    #         self._robot.power_on()
    #         self.stand()

    #         trajectory_point = (
    #             RobotCommandBuilder.create_arm_joint_trajectory_point(
    #                 joint_targets[0],
    #                 joint_targets[1],
    #                 joint_targets[2],
    #                 joint_targets[3],
    #                 joint_targets[4],
    #                 joint_targets[5],
    #             )
    #         )
    #         arm_joint_trajectory = arm_command_pb2.ArmJointTrajectory(
    #             points=[trajectory_point]
    #         )
    #         arm_command = self.make_arm_trajectory_command(arm_joint_trajectory)

    #         # Send the request
    #         cmd_id = self._robot_command_client.robot_command(arm_command)

    #         # Query for feedback to determine how long it will take
    #         feedback_resp = self._robot_command_client.robot_command_feedback(
    #             cmd_id
    #         )
    #         joint_move_feedback = (
    #             feedback_resp.feedback.synchronized_feedback.arm_command_feedback.arm_joint_move_feedback
    #         )
    #         time_to_goal: Duration = joint_move_feedback.time_to_goal
    #         time_to_goal_in_seconds: float = time_to_goal.seconds + (
    #             float(time_to_goal.nanos) / float(10**9)
    #         )
    #         time.sleep(time_to_goal_in_seconds)
    #         return True, 'Success'

    #     except Exception as e:
    #         return False, Text(e)
