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

from bosdyn.client import create_standard_sdk, ResponseError, RpcError, power
from bosdyn.client.auth import AuthResponseError
from bosdyn.client.async_tasks import AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.lease import ResourceAlreadyClaimedError, InvalidResourceError, NotAuthoritativeServiceError, LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_id import RobotIdClient
import bosdyn.client.util
from bosdyn.util import seconds_to_duration

from google.protobuf.timestamp_pb2 import Timestamp as PB2Timestamp
from google.protobuf.duration_pb2 import Duration as PB2Duration
from google.protobuf.message import Message as PB2Message

class SpotBaseWrapper():
    """Generic wrapper class to encompass release 1.1.4 API features as well as maintaining leases automatically"""
    def __init__(self):
        self._is_connected = False
        self._robot = None
        self._lease = None
        self._hostname = None

        # Clients
        self._robot_state_client = None
        self._robot_command_client = None
        self._power_client = None
        self._lease_client = None
        self._estop_client = None

    def connect(self, logger, hostname, rates = {}, callbacks = {}) -> bool:
        if self._is_connected:
            logger.info("Already connected to robot, no need to connect again")
            return True

        self._logger = logger
        self._hostname = hostname

        try:
            self._sdk = bosdyn.client.create_standard_sdk('ros_spot')
        except IOError as err:
            logger.error('Error creating SDK object ' + Text(err))
            return False

        self._robot = self._sdk.create_robot(hostname)

        logger.info("Authenticating")
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
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
        except Exception as e:
            logger.error('Unable to create client service: ' + Text(e))
            return False

        # Async Tasks
        self._robot_state_task = AsyncRobotState(self._robot_state_client, logger, rates.get("status.robot_state", 1.0), callbacks.get("robot_state", lambda:None))
        self._lease_task = AsyncLease(self._lease_client, logger, rates.get("status.lease", 1.0), callbacks.get("lease", lambda:None))
        self._idle_task = AsyncIdle(self._robot_command_client, logger, 10.0, self)

        self._estop_endpoint = None

        self._async_status_tasks = AsyncTasks([self._robot_state_task,
                                               self._lease_task,
                                               self._idle_task
                                              ])

        self._is_connected = True
        return True

    @property
    def robot(self):
        return self._robot
    
    @property
    def hostname(self):
        return self._hostname

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
