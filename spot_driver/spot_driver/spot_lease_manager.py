############################################################################################
#      Title     : spot_lease_manager.py
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
import atexit

from bosdyn.client import create_standard_sdk, ResponseError, RpcError, power
from bosdyn.client.auth import AuthResponseError
from bosdyn.client.async_tasks import AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.lease import ResourceAlreadyClaimedError, InvalidResourceError, NotAuthoritativeServiceError, LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, block_until_arm_arrives
from bosdyn.client.robot_id import RobotIdClient
import bosdyn.client.util
from bosdyn.util import seconds_to_duration
from bosdyn.api import estop_pb2

from google.protobuf.timestamp_pb2 import Timestamp as PB2Timestamp
from google.protobuf.duration_pb2 import Duration as PB2Duration
from google.protobuf.message import Message as PB2Message

# Type hint helpers
class EStopSystemStatusProto(type[estop_pb2.EstopSystemStatus]): pass

class DefaultLogger():
    """Generic print logger to act as default logger for the lease manager"""
    def info(self, msg):
        print(msg)

    def warn(self, msg):
        print(f"\033[33m{msg}\033[0m")

    def error(self, msg):
        print(f"\033[31m{msg}\033[0m")

    def fatal(self, msg):
        self.error(msg)

class SpotLeaseManager():
    """Generic manager class to maintain leases automatically"""
    def __init__(self):        
        # State
        self._is_connected = False
        self._robot = None
        self._lease = None
        self._hostname = None
        self._is_frozen = False
        self._lease_proto = None
        self._lease_query_future = None

        # Clients
        self._robot_state_client = None
        self._robot_command_client = None
        self._power_client = None
        self._lease_client = None
        self._lease_keepalive = None
        self._estop_client = None
        self._estop_endpoint = None
        self._estop_keepalive = None

        # Keep track of who is using the lease
        self._lease_owners = []

        # Register the safe power off function for emergency shutdown
        atexit.register(lambda: [self.disconnect(owner_id) for owner_id in self._lease_owners])

    def setLogger(self, logger):
        """Set the logger"""
        self._logger = logger

    def connect(self, hostname) -> bool:
        """
        Connect the lease manager to a Spot robot at address 'hostname'. Additionally creates
        a time-sync between the host computer and the robot and registers clients for robot
        lease, estop, state, command, and power.

        Args:
            hostname : IP address of the robot
            callbacks: A dict of callable functions of signature 'def func(FutureWrapper)'. 
                       In most cases, the FutureWrapper argument is not used and can be '_'
            rates    : The rates at which to call each of the callbacks

        Note:
            Valid keys for rates are ['status.lease'] and valid keys for callbacks are ['lease']

        Returns:
            Bool describing whether connection was successful
        """

        if self._is_connected:
            self.logger.info("Already connected to robot, no need to connect again")
            return True

        self._hostname = hostname

        try:
            self._sdk = bosdyn.client.create_standard_sdk('ros_spot')
        except IOError as err:
            self.logger.error('Error creating SDK object ' + Text(err))
            return False

        self._robot = self._sdk.create_robot(hostname)

        self.logger.info("Authenticating...")
        try:
            bosdyn.client.util.authenticate(self._robot)
        except RpcError as err:
            self.logger.error('Failed to communicate with robot {}: {}'.format(hostname, err.error_message))
            return False
        except AuthResponseError as err:
            self.logger.error('Authentication failed. ' + err.error_message)
            return False

        self.logger.info("Authentification successful, starting time sync...")
        self._robot.start_time_sync()
        self._robot.time_sync.wait_for_sync()

        # Spot service clients
        self.logger.info("Starting robot clients")
        try:
            self._robot_state_client: RobotStateClient = self._robot.ensure_client(RobotStateClient.default_service_name)
            self._robot_command_client: RobotCommandClient = self._robot.ensure_client(RobotCommandClient.default_service_name)
            self._power_client: PowerClient = self._robot.ensure_client(PowerClient.default_service_name)
            self._lease_client: LeaseClient = self._robot.ensure_client(LeaseClient.default_service_name)
            self._estop_client: EstopClient = self._robot.ensure_client(EstopClient.default_service_name)
        except Exception as e:
            self.logger.error('Unable to create client service: ' + Text(e))
            return False

        self._estop_endpoint = None
        self._is_connected = True
        self.logger.info("Robot connection established")
        return True

    @property
    def robot(self) -> bosdyn.client.Robot:
        return self._robot
    
    @property
    def hostname(self):
        return self._hostname

    @property
    def logger(self):
        """Return the logger"""
        return self._logger if self._logger is not None else DefaultLogger()

    @property
    def is_connected(self) -> bool:
        """Return boolean indicating if the lease manager is registered with a robot"""
        return self._is_connected

    @property
    def frozen(self) -> bool:
        """Return boolean indicating if the robot is currently allowed to move"""
        return self._is_frozen

    @property
    def ID(self):
        """Return robot's ID"""
        if not self._is_connected:
            return None
            
        return self._robot.get_id()

    @property
    def lease(self):
        """Return latest proto from the lease request"""
        return self._lease_proto
    
    @property
    def command_client(self):
        """Return the client used to pass commands to the robot"""
        return self._robot_command_client

    @property
    def time_skew(self) -> PB2Duration:
        """Return the time skew between local and spot time"""
        return self._robot.time_sync.endpoint.clock_skew
    
    @property
    def robot_time(self) -> PB2Timestamp:
        """Return the current time as a robot time protobuf timestamp"""
        return self._robot.time_sync.robot_timestamp_from_local_secs(time.time())
    
    @property
    def is_frozen(self) -> bool:
        """Return whether or not the robot is allowed to accept new command or move"""
        return self._is_frozen
    
    def setLeaseQueryResult(self, future: Future) -> None:
        self._lease_proto = future.result()

    def updateLeaseInfo(self) -> None:
        if self._lease_query_future is None or self._lease_query_future.done():
            self._lease_query_future = self._lease_client.list_leases_async()
            self._lease_query_future.add_done_callback(self.setLeaseQueryResult)
    
    def registerLeaseOwner(self, owner_id, force: bool = False) -> Tuple[bool, Text]:
        if self.isRegisteredLeaseOwner(owner_id):
            self.logger.warn(f"Lease already owned for object with id {owner_id}")
            return True, 'You already own this lease'

        (success, msg) = self.claim(force) if self._lease is None else (True, "Success")
        
        if success:
            self._lease_owners.append(owner_id)
            self.logger.info(f"Lease owner added with id {owner_id}. Total owners: {len(self._lease_owners)}")

        return success, msg

    def isRegisteredLeaseOwner(self, ID) -> bool:
        """Check to see if a particular object is a registered lease owner
        
        Args: 
            ID: The Python id of the object in question
        Returns:
            True if the object owns a lease, False otherwise 
        """
        return True if ID in self._lease_owners else False 

    def freeze(self) -> Tuple[bool, Text]:
        """Stop the robot and prevent it from making any further movements"""
        self._is_frozen = True
        try:
            self._robot_command_client.robot_command(RobotCommandBuilder.stop_command())
            return True, "Robot frozen"
        except Exception as e:
            return False, f"Error occured commanding the robot to stop: {e}. However the robot is still disabled from accepting any new commands"
        
    def unfreeze(self) -> None:
        self._is_frozen = False

    def robot_command(self, command_proto: PB2Message,
                       end_time_secs: float =None) -> Tuple[bool, Text, int]:
        """Generic non blocking function for sending commands to robots.

        Args:
            command_proto: robot_command_pb2 protobuf message to send to the robot.
                           Usually made with RobotCommandBuilder
            end_time_secs: (optional) Time-to-live for the command in seconds
        """
        if self._is_frozen:
            message = "Cannot issue a command to the robot while frozen"
            return False, message, None
        
        try:
            id = self._robot_command_client.robot_command(lease=None, command=command_proto, end_time_secs=end_time_secs)
            return True, "Success", id
        except Exception as e:
            return False, Text(e), None

    def robot_command_feedback(self, command_id: int):
        """Get feedback proto from the given command

        Args:
            command_id: ID of a previously issued command, return from a call to 'robot_command'
        
        Returns:
            The appropriate feedback message type for the given command, or None if no command was found
        """
        try:
            return self._robot_command_client.robot_command_feedback(command_id)
        except RpcError as ex:
            self.logger.warn(f"{ex}")

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

    def claim(self, force: bool = False) -> Tuple[bool, Text]:
        """Get a lease for the robot, a handle on the estop endpoint, and the ID of the robot."""
        try:
            got_lease, msg = self.getLease(force)
            if not got_lease:
                return False, msg
            self.resetEStop()
        except (ResponseError, RpcError) as err:
            return False, err.error_message
        
        return True, 'Success'

    def resetEStop(self) -> None:
        """Get keepalive for eStop"""
        self.logger.info("Creating EStop endpoint")
        if self._estop_keepalive is not None:
            self._estop_keepalive.shutdown()
            self._estop_keepalive = None

        self._estop_endpoint = EstopEndpoint(self._estop_client, 'ros', 9.0)
        self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
        self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)

    def eStopStatus(self) -> EStopSystemStatusProto:
        """Get the status for the EStop client"""
        return self._estop_client.get_status()

    def assertEStop(self, severe=True) -> Tuple[bool, str]:
        """Forces the robot into eStop state.

        Args:
            severe: Default True - If true, will cut motor power immediately.  If false, will try to settle the robot on the ground first
        """
        try:
            if severe:
                self._estop_endpoint.stop()
                self.logger.error("Severe EStop triggered")
            else:
                self._estop_endpoint.settle_then_cut()
                self.logger.warn("EStop triggered")
        except Exception as e:
            return False, f"{e}"

        return True, "Successfully triggered e-stop"

    def _releaseEStop(self) -> None:
        """Stop eStop keepalive"""
        if self._estop_keepalive:
            self._estop_keepalive.stop()
            self._estop_keepalive = None
            self._estop_endpoint = None

    def getLease(self, force: bool = False) -> Tuple[bool, Text]:
        """Get a lease for the robot and keep the lease alive automatically."""
        try:
            self.logger.info("Obtaining lease...")
            self._lease = self._lease_client.acquire() if not force else self._lease_client.take()
        except (ResourceAlreadyClaimedError, InvalidResourceError, NotAuthoritativeServiceError) as err:
            self.logger.error(f"Unable to obtain lease: {Text(err.error_message)}")
            return False, err.error_message
        
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)
        self.logger.info("Lease acquired")
        return True, 'Success'

    def _releaseLease(self) -> None:
        """Return the lease on the body."""
        if self._lease:
            self._lease_client.return_lease(self._lease)
            self._lease = None
            self.logger.info("Shutting down lease keepalive")
            if self._lease_keepalive is not None:
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None

    def safe_shut_down(self):
        if self.robot.has_arm():
            _, _, cmd_id = self.robot_command(RobotCommandBuilder.arm_stow_command())
            try:
                block_until_arm_arrives(self._robot_command_client, cmd_id, timeout_sec=5.0)
            except:
                pass
        powered_off, msg = self.safe_power_off()
        if powered_off:
            self._releaseLease()
            self._releaseEStop()
        else:
            self.logger.warn(f"{msg}") 
        self.robot.time_sync.stop()
        self._is_connected = False

    def disconnect(self, owner_id) -> bool:
        try:
            self._lease_owners.remove(owner_id)
            self.logger.info(f"Released lease for owner {owner_id}")
            if len(self._lease_owners) == 0:
                self.logger.info("No more lease owners, powering off robot and releasing lease")
                self.safe_shut_down()
            self.logger.info("Successfully disconnected from lease owner")
            return True
        except ValueError:
            self.logger.warn("A non-owner just attempted to disconnect. Make sure to call registerLeaseOwner when first connecting to the LeaseManager")
            return False

    def power_on(self) ->  Tuple[bool, Text]:
        """Enable the motor power if e-stop is enabled."""
        try:
            power.power_on_motors(self._power_client)
            return True, 'Success'
        except Exception as e:
            return False, Text(e)

    def safe_power_off(self) -> Tuple[bool, Text]:
        """Stop the robot's motion and sit if possible.  Once sitting, disable motor power."""
        self.robot.power_off(cut_immediately=False, timeout_sec=20)
        if self.robot.is_powered_on():
            return False, "Robot power off failed."
        return not self.robot.is_powered_on(), "Success"
