############################################################################################
#      Title     : spot_body_wrapper.py
#      Project   : spot_ros
#      Copyright : Copyright© The University of Texas at Austin, 2024. All rights reserved.
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
from asyncio import Future

from bosdyn.api import header_pb2
from bosdyn.api.docking import docking_pb2
from bosdyn.api.spot import robot_command_pb2
from bosdyn.geometry import EulerZXY

from bosdyn.client.common import FutureWrapper
from bosdyn.client.async_tasks import AsyncTasks
from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.point_cloud import build_pc_request
from bosdyn.client.spot_cam.audio import AudioClient
from bosdyn.client.robot_command import RobotCommandBuilder

from google.protobuf.timestamp_pb2 import Timestamp as PB2Timestamp
from google.protobuf.duration_pb2 import Duration as PB2Duration

from .spot_lease_manager import SpotLeaseManager
from .type_hint_helpers import *

class SpotBodyWrapper():
    """Generic wrapper class to encompass release 4.0.2 API features"""
    def __init__(self, logger, hostname, has_eap_2: bool = False, has_cam_payload: bool = False):
        self._logger = logger
        self._hostname = hostname

        self._is_connected = False
        self._has_eap_2 = has_eap_2
        self._has_cam_payload = has_cam_payload
        self._lease_manager = None

        """ State futures """
        self._robot_state_future: FutureWrapper = None
        self._robot_state_proto = None

        self._robot_id = None
        self._is_sitting = True
        self._is_standing = False
        self._mobility_params = RobotCommandBuilder.mobility_params()
        self._is_moving = False
        self._last_docking_command = None
        self._last_stand_command = None
        self._last_sit_command = None
        self._last_trajectory_command = None
        self._last_trajectory_command_precise = None
        self._last_velocity_command_time = None

    def connect(self, lease_manager: SpotLeaseManager, rates = {}, callbacks = {}) -> bool:
        """
        Connect the lease manager to a Spot robot at address 'hostname' if it is not already connected. 
        Additionally registers self as a lease owner with this lease manager registers clients for robot
        lease, estop, state, command, and power.

        Args:
            hostname : IP address of the robot
            callbacks: A dict of callable functions of signature 'def func(FutureWrapper)'. 
                       In most cases, the FutureWrapper argument is not used and can be '_'
            rates    : The rates at which to call each of the callbacks

        Note:
            Valid keys for rates are ['status.robot_state'] 
            and valid keys for callbacks are ['robot_state']

        Returns:
            Bool describing whether connection was successful
        """
        
        if lease_manager is None:
            self.logger.fatal("Cannot connect to robot without a valid lease manager object")
            return False
        
        # Have the lease manager connect to the robot
        self._lease_manager = lease_manager
        if not self._lease_manager.is_connected:
            self._lease_manager.setLogger(self._logger)
            if not self._lease_manager.connect(self._hostname, rates, callbacks):
                return False

        self._robot_id = self._lease_manager.ID

        # Spot service clients
        try:
            self._docking_client = self._lease_manager.robot.ensure_client(DockingClient.default_service_name) 

            if self._has_eap_2:
                self._pointcloud_client = self._lease_manager.robot.ensure_client('velodyne-point-cloud')


        except Exception as e:
            self.logger.error('Unable to create client service: ' + Text(e))
            return False

        if self._has_cam_payload:
            try:
                self._audio_client = self._lease_manager.robot.ensure_client(AudioClient.default_service_name)
            except Exception as e:
                self.logger.error('Unable to create client service: ' + Text(e))
                return False

        sensor_tasks = []        

        # Optionally populate pointcloud data asynchronously
        if self._has_eap_2 and 'point_cloud' in callbacks:
            # Create point cloud requests
            point_cloud_requests = []
            point_cloud_sources = {'velodyne-point-cloud'}
            for source in point_cloud_sources:
                point_cloud_requests.append(build_pc_request(source))
            self._pointcloud_task = AsyncPointCloudService(self._pointcloud_client, self.logger, rates.get("sensors.point_cloud", 1.0), callbacks.get("point_cloud", lambda:None), point_cloud_requests)
            sensor_tasks.append(self._pointcloud_task)

        self._async_sensor_tasks = AsyncTasks(sensor_tasks)

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
        return self._robot_id

    @property
    def robot_state(self):
        """Return latest proto from the robot state response"""
        return self._robot_state_proto

    @property
    def lease(self):
        """Return latest proto from the _lease_task"""
        return self._lease_manager.lease

    @property
    def point_clouds(self):
        """Return the latest proto from teh _pointcloud_task"""
        return self._pointcloud_task.proto

    @property
    def is_sitting(self) -> bool:
        """Return boolean of sitting state"""
        return self._is_sitting

    @property
    def is_standing(self) -> bool:
        """Return boolean of standing state"""
        return self._is_standing

    @property
    def is_moving(self) -> bool:
        """Return boolean of walking state"""
        return self._is_moving

    @property
    def time_skew(self) -> PB2Duration:
        """Return the time skew between local and spot time"""
        return self._lease_manager.time_skew
    
    def robotToLocalTime(self, timestamp: PB2Timestamp) -> PB2Timestamp:
        """Return the robot time in local time as a proto timestamp"""
        return self._lease_manager.robotToLocalTime(timestamp)

    def setStateResult(self, future: Future) -> None:
        self._robot_state_proto = future.result()

    def udpateState(self) -> None:
        """Update the robot state"""
        if self._robot_state_future is None or self._robot_state_future.done():
            self._robot_state_future = self._lease_manager._robot_state_client.get_robot_state_async()
            self._robot_state_future.add_done_callback(self.setStateResult)

    def updateSensorTasks(self) -> None:
        """Loop through the sensor query periodic tasks and update their data if needed."""
        self._async_sensor_tasks.update()

    def claim(self, force: bool = False) -> bool:
        """Add this driver as an EStop and Lease owner of the lease manager"""
        if self._lease_manager is None:
            self.logger.warn("Cannot claim a lease without first connecting to a LeaseManager!")
            return False
        
        self._lease_manager.registerLeaseOwner(id(self), force)
        return True        
    
    def release(self) -> None:
        """Return the lease on the body"""
        try:
            self.sit()
            self._lease_manager.disconnect(id(self))
        except Exception as err:
            return False, Text(err)

        return True, 'Success'

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
    
    def freeze(self) -> Tuple[bool, Text]:
        """Stop the robot's motion and prevent it from accepting any new commands"""
        success, message = self._lease_manager.freeze()
        return success, message
    
    def unfreeze(self) -> None:
        """Allow the robot to accept motion commands"""
        self._lease_manager.unfreeze()

    def self_right(self) -> Tuple[bool, Text]:
        """Have the robot self-right itself."""
        response = self._lease_manager.robot_command(RobotCommandBuilder.selfright_command())
        return response[0], response[1]

    def sit(self) -> Tuple[bool, Text]:
        """Stop the robot's motion and sit down if able."""
        if self._lease_manager.robot.has_arm():
            self._lease_manager.robot_command(RobotCommandBuilder.arm_stow_command())

        success, msg, cmd_id = self._lease_manager.robot_command(RobotCommandBuilder.synchro_sit_command())
        self._last_sit_command = cmd_id
        return success, msg

    def stand(self, monitor_command=True) -> Tuple[bool, Text]:
        """If the e-stop is enabled, and the motor power is enabled, stand the robot up. This command is NON-BLOCKING!"""
        success, msg, cmd_id = self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))
        if monitor_command:
            self._last_stand_command = cmd_id
        return success, msg

    def dock(self, dock_id) -> Tuple[bool, Text]:
        """Dock the robot to the docking station with fiducial ID [dock_id]."""
        if self._lease_manager.frozen:
            return False, "Cannot issue a command to the robot while frozen"
        
        try:
            # Dock the robot
            self.last_docking_command = dock_id
            blocking_dock_robot(self._lease_manager.robot, dock_id)
            self.last_docking_command = None
        except Exception as e:
            return False, Text(e)
        return True, 'Success'

    def undock(self, timeout: float = 20.0) -> Tuple[bool, Text]:
        """Power motors on and undock the robot from the station."""
        current_dock_state = self.get_docking_state()
        undocked: bool = current_dock_state.status == docking_pb2.DockState.DockedStatus.DOCK_STATUS_UNDOCKED
        undocking: bool = current_dock_state.status == docking_pb2.DockState.DockedStatus.DOCK_STATUS_UNDOCKING
        if undocked or undocking:
            return True, 'Already undocked'
        
        elif self._lease_manager.frozen:
            return False, "Cannot issue a command to the robot while frozen"
        
        try:
            # Undock the robot
            blocking_undock(self._lease_manager.robot, timeout)
        except Exception as e:
            return False, Text(e)
        return True, 'Success'
    
    def walk_to(self, target_pose_in_odom: SE2PoseProto, max_duration: float) -> Tuple[bool, Text]:
        navigate_command = RobotCommandBuilder.synchro_se2_trajectory_command(
            goal_se2=target_pose_in_odom,
            frame_name=ODOM_FRAME_NAME
        )

        success, message, command_id = self._lease_manager.robot_command(navigate_command, end_time_secs=time.time() + max_duration)
        return success, message, command_id

    def get_docking_state(self, **kwargs) -> DockStateProto:
        """Get docking state of robot."""
        state = self._docking_client.get_docking_state(**kwargs)
        return state

    def set_mobility_params(self,
                            body_height_offset: float = 0.0,
                            footprint_R_body: EulerZXY = EulerZXY(),
                            locomotion_hint: int = robot_command_pb2.LocomotionHint.Value('HINT_AUTO'),
                            stair_hint: bool = False,
                            external_force_params: BodyExternalParamsProto = None) -> None:
        """Define body, locomotion, and stair parameters.

        Args:
            body_height: Body height offset from nominal position in meters
            footprint_R_body: (EulerZXY) - The orientation of the body frame with respect to the footprint frame (gravity aligned framed with yaw computed from the stance feet)
            locomotion_hint: Locomotion hint
            stair_hint: Boolean to define stair motion
        """
        self._mobility_params = RobotCommandBuilder.mobility_params(body_height_offset, footprint_R_body, locomotion_hint, stair_hint, external_force_params)

    def get_mobility_params(self) -> MobilityParamsProto:
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
    
    def update_idle_state(self) -> None:
        if self._last_stand_command is not None:
            try:
                response = self._lease_manager.command_client.robot_command_feedback(self._last_stand_command)
                self._is_sitting = False
                if (response.feedback.synchronized_feedback.mobility_command_feedback.stand_feedback.status ==
                        basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING):
                    self._is_standing = True
                    self._last_stand_command = None
                else:
                    self._is_standing = False
            except (ResponseError, RpcError) as e:
                self._logger.error(f"Error when getting robot command feedback: {e}")
                self._last_stand_command = None

        if self._last_sit_command is not None:
            try:
                self._is_standing = False
                response = self._lease_manager.command_client.robot_command_feedback(self._last_sit_command)
                if (response.feedback.synchronized_feedback.mobility_command_feedback.sit_feedback.status ==
                        basic_command_pb2.SitCommand.Feedback.STATUS_IS_SITTING):
                    self._is_sitting = True
                    self._last_sit_command = None
                else:
                    self._is_sitting = False
            except (ResponseError, RpcError) as e:
                self._logger.error(f"Error when getting robot command feedback: {e}")
                self._last_sit_command = None

        if self._last_velocity_command_time != None:
            if time.time() < self._last_velocity_command_time:
                self._is_moving = True
            else:
                self._last_velocity_command_time = None

        if self._last_trajectory_command != None:
            try:
                response = self._lease_manager.command_client.robot_command_feedback(self._last_trajectory_command)
                status = response.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback.status
                # STATUS_AT_GOAL always means that the robot reached the goal. If the trajectory command did not
                # request precise positioning, then STATUS_NEAR_GOAL also counts as reaching the goal
                if status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL or \
                    (status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_NEAR_GOAL and
                     not self._last_trajectory_command_precise):
                    self._at_goal = True
                    # Clear the command once at the goal
                    self._last_trajectory_command = None
                elif status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_GOING_TO_GOAL:
                    self._is_moving = True
                elif status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_NEAR_GOAL:
                    self._is_moving = True
                    self._near_goal = True
                else:
                    self._last_trajectory_command = None
            except (ResponseError, RpcError) as e:
                self._logger.error(f"Error when getting robot command feedback: {e}")
                self._last_trajectory_command = None

        # TODO: Verify that this logic is correct. If I understand this, this line will almost never (or perhaps actually never) execute
        if (self.is_standing and not self.is_moving
                    and not self._lease_manager.frozen
                    and self._last_trajectory_command is not None
                    and self._last_stand_command is not None
                    and self._last_velocity_command_time is not None
                    and self._last_docking_command is not None
                    and self.lease is not None):            
            self.stand(True)

    def sassy_confused(self) -> Tuple[bool, str]:
        """Makes Spot look confused in a bit of a sassy way"""
        try:
            # Lower and rotate Spot's body
            roll = EulerZXY(yaw=0.0, roll=0.4, pitch=0.0)
            body_height = -0.1
            self.set_mobility_params(body_height_offset=body_height,footprint_R_body=roll)

            # Ensure params are set
            assert self._mobility_params is not None, "Mobility parameters not set!"

            self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))
            
            # Maintain the pose for 1 seconds
            time.sleep(1)

            # Restore to normal standing pose
            self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command())


            return True, "Spot successfully completed low rotate and returned to normal stance."

        except Exception as e:
            return False, f"Failed to execute low rotate sequence: {e}"
        
    def no_nod(self) -> Tuple[bool, str]:
        """Makes Spot do a quick "no" gesture."""
        try:
            yaw_sequence = [0.15, -0.15, 0.15, -0.15, 0.15, -0.15]

            for yaw_element in yaw_sequence:

                nod = EulerZXY(yaw=yaw_element, roll=0.0, pitch=0.1)
                self.set_mobility_params(body_height_offset=0.0, footprint_R_body=nod)
                
                # Execute yaw command
                self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))
            
                # Maintain the pose for 1 seconds
                time.sleep(0.25)

            # Restore to normal standing pose
            self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command())

            return True, "Spot successfully performed a 'no nod' gesture"

        except Exception as e:
            return False, f"Failed to execute 'no nod' gesture: {e}"

    def water_shakeoff(self) -> Tuple[bool, str]:
        """Makes Spot do a quick "water shakeoff" gesture."""
        try:
            sequence = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0]

            for element in sequence:

                shake = EulerZXY(yaw=0.1*element, roll=0.5*element, pitch=0.0)
                self.set_mobility_params(body_height_offset=-0.15, footprint_R_body=shake)
                
                # Execute water_shakeoff command
                self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))
            
                # Maintain the pose for 1 seconds
                time.sleep(0.23)

            # Restore to normal standing pose
            self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command())

            return True, "Spot successfully performed a 'water shakeoff' gesture"

        except Exception as e:
            return False, f"Failed to execute 'water shakeoff' gesture: {e}"

    def serious_stance(self) -> Tuple[bool, str]:
        """Makes Spot perform a "serious" gesture stance."""
        try:

            pose = EulerZXY(yaw=0.0, roll=0.0, pitch=0.2)
            self.set_mobility_params(body_height_offset=-0.19, footprint_R_body=pose)
            
            # Execute stance command
            self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))

            return True, "Spot successfully performed a 'serious stance' gesture"

        except Exception as e:
            return False, f"Failed to execute 'serious stance' gesture: {e}"

    def perform_gesture(self, gesture_sequence) -> Tuple[bool, str]:
        """Makes Spot perform a gesture sequence with validation"""

        # Define the valid bounds for each gesture component
        BOUNDS = {
            "yaw": (-0.6, 0.6),           # Yaw range in radians
            "roll": (-0.6, 0.6),          # Roll range in radians
            "pitch": (-0.6, 0.6),         # Pitch range in radians
            "body_height": (-0.2, 0.2),   # Height offset in meters
            "pose_duration": (0.0, 10.0)  # Duration in seconds (min/max duration)
            }

        try:
            for idx, gesture in enumerate(gesture_sequence):
                # Extract individual gesture components
                yaw = gesture.yaw
                roll = gesture.roll
                pitch = gesture.pitch
                body_height = gesture.body_height
                pose_duration = gesture.pose_duration

                # Build a dictionary for easier validation and clearer messages
                gesture_dict = {
                    "yaw": yaw,
                    "roll": roll,
                    "pitch": pitch,
                    "body_height": body_height,
                    "pose_duration": pose_duration
                }

                # Validate each value against its bounds
                for label, value in gesture_dict.items():
                    min_val, max_val = BOUNDS[label]
                    if not (min_val <= value <= max_val):
                        return False, (
                            f"Gesture {idx} has an invalid {label} value: {value} "
                            f"(allowed range: {min_val} to {max_val})"
                        )

                # If all values are within bounds, proceed with execution
                posture = EulerZXY(yaw=yaw, roll=roll, pitch=pitch)
                self.set_mobility_params(body_height_offset=body_height,footprint_R_body=posture)

                # Execute the gesture
                self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command(params=self._mobility_params))
                
                # Maintain the pose for the desired duration
                time.sleep(pose_duration)

            # Restore Spot to its normal standing pose
            self._lease_manager.robot_command(RobotCommandBuilder.synchro_stand_command())
            
            return True, "Spot successfully performed the gesture sequence"

        except Exception as e:
            return False, f"Failed to execute gesture sequence: {e}"
