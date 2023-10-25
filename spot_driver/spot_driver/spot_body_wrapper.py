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

from .spot_lease_manager import SpotLeaseManager

class SpotBodyWrapper():
    """Generic wrapper class to encompass release 1.1.4 API features"""
    def __init__(self, logger, hostname, has_cam_payload: bool = False):
        self._logger = logger
        self._hostname = hostname

        self._is_connected = False
        self._has_cam_payload = has_cam_payload
        self._lease_manager = None

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

    def connect(self, lease_manager: SpotLeaseManager, rates = {}, callbacks = {}) -> bool:
        if lease_manager is None:
            self.logger().fatal("Cannot connect to robot without a valid base wrapper object")
            return False
        
        # Have the base wrapper connect to the robot
        self._lease_manager = lease_manager
        if not self._lease_manager.is_connected:
            self._lease_manager.setLogger(self._logger)
            if not self._lease_manager.connect(self._hostname, rates, callbacks):
                return False

        front_image_sources = {'frontleft_fisheye_image', 'frontright_fisheye_image', 'frontleft_depth', 'frontright_depth'}
        side_image_sources = {'left_fisheye_image', 'right_fisheye_image', 'left_depth', 'right_depth'}
        rear_image_sources = {'back_fisheye_image', 'back_depth'}
        hand_image_sources = {'hand_image', 'hand_depth', 'hand_color_image', 'hand_depth_in_hand_color_frame'}

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

        # Spot service clients
        try:
            self._image_client = self._lease_manager.robot.ensure_client(ImageClient.default_service_name)
            self._docking_client = self._lease_manager.robot.ensure_client(DockingClient.default_service_name) 
        except Exception as e:
            self.logger.error('Unable to create client service: ' + Text(e))
            return False

        if self._has_cam_payload:
            try:
                self._audio_client = self._lease_manager.robot.ensure_client(AudioClient.default_service_name)
            except Exception as e:
                self.logger.error('Unable to create client service: ' + Text(e))
                return False

        # Async Tasks
        self._front_image_task = AsyncImageService(self._image_client, self.logger, rates.get("sensors.front_image", 1.0), callbacks.get("front_image", lambda:None), front_image_requests)
        self._side_image_task = AsyncImageService(self._image_client, self.logger, rates.get("sensors.side_image", 1.0), callbacks.get("side_image", lambda:None), side_image_requests)
        self._rear_image_task = AsyncImageService(self._image_client, self.logger, rates.get("sensors.rear_image", 1.0), callbacks.get("rear_image", lambda:None), rear_image_requests)
        self._hand_image_task = AsyncImageService(self._image_client, self.logger, rates.get("sensors.hand_image", 1.0), callbacks.get("hand_image", lambda:None), hand_image_requests)
        self._idle_task = AsyncIdle(self._lease_manager.command_client, self.logger, 10.0, self)
        self._robot_state_task = AsyncRobotState(self._lease_manager._robot_state_client, self.logger, rates.get("status.robot_state", 1.0), callbacks.get("robot_state", lambda:None))

        self._async_sensor_tasks = AsyncTasks([self._front_image_task,
                                               self._side_image_task,
                                               self._rear_image_task,
                                               self._hand_image_task
                                               ])
        
        self._async_idle_task  = AsyncTasks([self._idle_task])
        self._async_state_task = AsyncTasks([self._robot_state_task])

        self._is_connected = True
        return True

    @property
    def logger(self):
        """Return this wrapper's logger"""
        return self._lease_manager.logger

    @property
    def is_connected(self) -> bool:
        """Return boolean indicating if the wrapper initialized successfully"""
        return self._is_connected

    @property
    def robot_id(self):
        """Return robot's ID"""
        if not self._is_connected:
            return None
            
        return self._lease_manager.robot.get_id()

    @property
    def robot_state(self):
        """Return latest proto from the _robot_state_task"""
        return self._robot_state_task.proto

    @property
    def lease(self):
        """Return latest proto from the _lease_task"""
        return self._lease_manager.lease

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
        return self._lease_manager.time_skew
    
    def robotToLocalTime(self, timestamp: PB2Timestamp) -> PB2Timestamp:
        if self._lease_manager is not None:
            return self._lease_manager.robotToLocalTime(timestamp)

    def updateStateTasks(self) -> None:
        """Update the robot state"""
        self._async_state_task.update()

    def updateIdleTasks(self) -> None:
        """Update the idle task"""
        self._async_idle_task.update()

    def updateSensorTasks(self) -> None:
        """Loop through the sensor query periodic tasks and update their data if needed."""
        self._async_sensor_tasks.update()

    def claim(self) -> bool:
        """Add this driver as an EStop and Lease owner of the base wrapper"""
        if self._lease_manager is None:
            self.logger.warn("Cannot claim a lease without first connecting to a BaseWrapper!")
            return False
        
        self._lease_manager.registerLeaseOwner(id(self))
        return True        
    
    def release(self) -> None:
        """Return the lease on the body"""
        try:
            self.sit()
            self._lease_manager.disconnect(id(self))
        except Exception as err:
            return False, Text(err)

        return True, 'Success'

    # def disconnect(self) -> None:
    #     """Release control of robot as gracefully as posssible."""
    #     if self._lease_manager.robot is None:
    #         return

    #     if self._lease_manager.robot.time_sync:
    #         self._lease_manager.robot.time_sync.stop()
    #     self.release()
    def power_on(self) -> Tuple[bool, Text]:
        """Power on the robot's motors"""
        success, response = self._lease_manager.power_on()
        return success, response

    def power_off(self) -> Tuple[bool, Text]:
        """Safely power off the robot"""
        success, response = self._lease_manager.safe_power_off()
        return success, response

    def stop(self) -> Tuple[bool, Text]:
        """Stop the robot's motion."""
        response = self._lease_manager.robot_command(RobotCommandBuilder.stop_command())
        return response[0], response[1]

    def self_right(self) -> Tuple[bool, Text]:
        """Have the robot self-right itself."""
        response = self._lease_manager.robot_command(RobotCommandBuilder.selfright_command())
        return response[0], response[1]

    def sit(self) -> Tuple[bool, Text]:
        """Stop the robot's motion and sit down if able."""
        # self.arm_stow()
        response = self._lease_manager.robot_command(RobotCommandBuilder.synchro_sit_command())
        self._last_sit_command = response[2]
        return response[0], response[1]

    def stand(self, monitor_command=True) -> Tuple[bool, Text]:
        """If the e-stop is enabled, and the motor power is enabled, stand the robot up."""
        response = self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))
        if monitor_command:
            self._last_stand_command = response[2]
        return response[0], response[1]

    def dock(self, dock_id) -> Tuple[bool, Text]:
        """Dock the robot to the docking station with fiducial ID [dock_id]."""
        try:
            # Make sure we're powered on and standing
            self._lease_manager.robot.power_on()
            self.stand()
            # Dock the robot
            self.last_docking_command = dock_id
            blocking_dock_robot(self._lease_manager.robot, dock_id)
            self.last_docking_command = None
        except Exception as e:
            return False, Text(e)
        return True, 'Success'

    def undock(self, timeout: float = 20.0) -> Tuple[bool, Text]:
        """Power motors on and undock the robot from the station."""
        try:
            # Make sure we're powered on
            self._lease_manager.robot.power_on()

            # Undock the robot
            blocking_undock(self._lease_manager.robot, timeout)
        except Exception as e:
            return False, Text(e)
        return True, 'Success'

    def get_docking_state(self, **kwargs) -> docking_pb2.DockState:
        """Get docking state of robot."""
        state = self._docking_client.get_docking_state(**kwargs)
        return state

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
        self._lease_manager.robot_command(RobotCommandBuilder.synchro_velocity_command(
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


