import sys
import bosdyn.client.util
from bosdyn.client import create_standard_sdk, RpcError
from bosdyn.client.auth import AuthResponseError
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient

from rclpy import Node, utilities as rclpy_util
from scripts.follow_joint_trajectory_action_server import FollowJointTrajectoryActionServer
from spot_driver.spot_ros import SpotROS 

class SpotDriverCombined(Node):
    
    def __init__(self):
        # Robot API entry point
        self._sdk = None
        self._robot = None

        # Clients
        self._estop_client = None
        self._lease_client = None

        # Drivers
        self._body_driver = None
        self._arm_driver  = None

    def connect(self, hostname) -> bool:
        # Create an SDK object to get the robot
        try:
            self._sdk = create_standard_sdk("spot_ros")
            self._robot = self._sdk.create_robot(hostname)
            self._robot.start_time_sync()
        except IOError as err:
            self.get_logger().fatal(f"Error creating Bosdyn SDK object: {str(err)}")
            return False
        
        # Authenticate and log into the robot
        try:
            bosdyn.client.util.authenticate(self._robot)
        except RpcError as err:
            self.get_logger().fatal(f"Failed to communicate with robot {hostname}: {err.error_message}")
            return False
        except AuthResponseError as err:
            self.get_logger().fatal(f"Authentication failed. {err.error_message}")
            return False
        
        # Establish mission critical clients
        self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
        self._lease_client = self._robot.ensure_client(LeaseClient.default_service_name)

        # Create the ROS wrappers for the body and the arm
        argv = rclpy_util.remove_ros_args(args=sys.argv)[1:]
        self._body_driver = SpotROS()

        if self._robot.has_arm():
            self._arm_driver = FollowJointTrajectoryActionServer(argv)
