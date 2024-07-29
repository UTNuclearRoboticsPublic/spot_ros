from enum import Enum

import time
import rclpy
from asyncio import Future

import rclpy.duration
from rclpy.node import Node
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Joy
from spot_msgs.srv import Dock
from spot_msgs.msg import Feedback, ManipulatorState
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, Quaternion
from scipy.spatial.transform import Rotation

# Requires the controller switch to be in mode "D"
class LogitechButtons(Enum):
    A  = 1
    B  = 2
    X  = 0
    Y  = 3
    LB = 4
    RB = 5
    LT = 6
    RT = 7
    BACK  = 8
    START = 9
    LEFT_STICK  = 10
    RIGHT_STICK = 11

class LogitechAxes(Enum):
    LEFT_HORIZONTAL  = 0
    LEFT_VERTICAL    = 1
    RIGHT_HORIZONTAL = 2
    RIGHT_VERTICAL   = 3
    DPAD_HORIZONTAL  = 4
    DPAD_VERTICAL    = 5

def interpolate(val: float, input_range: list, output_range: list):
    return output_range[0] + (val - input_range[0])/(input_range[1] - input_range[0]) * (output_range[1] - output_range[0])

class SpotJoyUtils(Node):
    def __init__(self):
        super().__init__("spot_joy_util_node")

        # Robot state
        self._docked = True
        self._arm_stowed = True
        self._sitting = False
        self._gripper_closed = True

        # Subscribe to the feedback topic to monitor dock state
        self._feedback_sub = self.create_subscription(Feedback, '/spot_driver/status/feedback', self.updateState, 10)
        self._arm_feedback_sub = self.create_subscription(ManipulatorState, '/spot_manipulation_driver/manipulator_state', self.updateArmState, 10)

        exclusive_group = MutuallyExclusiveCallbackGroup()
        self.lease_client = self.create_client(Trigger, "/spot_driver/claim", callback_group=exclusive_group)
        self.release_client = self.create_client(Trigger, "/spot_driver/release", callback_group=exclusive_group)
        self.estop_client_gentle = self.create_client(Trigger, "/spot_driver/estop/gentle")
        self.estop_client_hard = self.create_client(Trigger, "/spot_driver/estop/hard")
        self.dock_client = self.create_client(Dock, "/spot_driver/dock", callback_group=exclusive_group)
        self.undock_client = self.create_client(Trigger, "/spot_driver/undock", callback_group=exclusive_group)
        self.power_on_client = self.create_client(Trigger, "/spot_driver/power_on", callback_group=exclusive_group)
        self.stand_client = self.create_client(Trigger, "/spot_driver/stand", callback_group=exclusive_group)
        self.sit_client = self.create_client(Trigger, "/spot_driver/sit", callback_group=exclusive_group)
        self.unstow_client = self.create_client(Trigger, "/spot_manipulation_driver/unstow_arm", callback_group=exclusive_group)
        self.stow_client = self.create_client(Trigger, "/spot_manipulation_driver/stow_arm", callback_group=exclusive_group)
        self.gripper_open_client = self.create_client(Trigger, "/spot_manipulation_driver/open_gripper", callback_group=exclusive_group)
        self.gripper_close_client = self.create_client(Trigger, "/spot_manipulation_driver/close_gripper", callback_group=exclusive_group)

        self.body_pose_pub = self.create_publisher(Pose, "/spot_driver/body_pose", 10, callback_group=exclusive_group)

        # E-Stop
        self._estop_future: Future | None = None
        self.estop_sub = self.create_subscription(Joy, "/joy", self.estopJoyCallback, 10) # no exclusive group so it has priority

        exclusive_group_2 = MutuallyExclusiveCallbackGroup()
        self.loop = self.create_timer(0.1, self.timerCallback, callback_group=exclusive_group_2)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joyCallback, 10, callback_group=exclusive_group_2)
        self.action = None

        self._body_offset = 0.0
        self._body_yaw    = 0.0
        self._body_pitch  = 0.0

        self.get_logger().info("Spot joy node setup complete")

    def updateState(self, msg: Feedback):
        self._docked = msg.docked
        self._sitting = msg.sitting

    def updateArmState(self, msg: ManipulatorState):
        self._arm_stowed = (msg.stow_state == ManipulatorState.STOWSTATE_STOWED) 
        self._gripper_closed = (msg.gripper_open_percentage < 70.0)
        
    def verifyServer(self, client) -> bool:
        if not client.wait_for_service(1):
            self.get_logger().warn(f"Service for action \"{self.action}\" is not available, cancelling request")
            self.action = None
            return False
        return True

    def timerCallback(self):        
        if self._estop_future is not None:
            if self._estop_future.done():
                resp: Trigger.Response = self._estop_future.result()
                self._logger.info(resp.message)
                self._estop_future = None
                return
            
        if self.action is None:
            return
        elif self.action == "Dock":
            self.get_logger().info("Toggling dock")
            self.dockRobot()
        elif self.action == "ToggleStand":
            self.get_logger().info("Toggling stand")
            self.toggleStand()
        elif self.action == "ToggleGripper":
            self.get_logger().info("Toggling gripper")
            self.toggleGripper()
        elif self.action == "BodyPoseControl":
            pose_command = Pose()
            q = Rotation.from_euler(seq="ZYX", angles=[self._body_yaw, self._body_pitch, 0.0], degrees=True).as_quat()
            pose_command.position.z = self._body_offset
            pose_command.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            self.body_pose_pub.publish(pose_command)
        else:
            if self.action == "Claim":
                self.get_logger().info("Claiming lease")
                client = self.lease_client
            elif self.action == "Release":
                self.get_logger().info("Releasing lease")
                client = self.release_client
            elif self.action == "Sit":
                self.get_logger().info("Sitting")
                client = self.sit_client
            elif self.action == "Stand":
                self.get_logger().info("Standing")
                client = self.stand_client
            elif self.action == "PowerOn":
                self.get_logger().info("Powering on")
                client = self.power_on_client
            elif self.action == "Undock":
                self.get_logger().info("Undocking robot")
                client = self.undock_client
            elif self.action == "ArmStow":
                self.get_logger().info("Stowing arm")
                client = self.stow_client
            elif self.action == "ArmUnstow":
                self.get_logger().info("Unstowing arm")
                client = self.unstow_client

            self.triggerClient(client)

        self.action = None


    def joyCallback(self, data: Joy):
        buttons = data.buttons
        axes    = data.axes

        # We need the controller in "D" mode, not "X" mode
        if len(axes) != 6:
            self.get_logger().warn("Logitech controller in wrong working mode. Please flip the switch on the back", throttle_duration_sec=1.0)
            return

        # If both the start button is pressed, try to claim a lease
        if buttons[LogitechButtons.START.value]:
            self.action = "Claim"
            return
        
        # If the back button is pressed, release the lease on the robot
        if buttons[LogitechButtons.BACK.value]:
            self.action = "Release"
            return

        # If both directional sticks are pressed, undock or dock the robot
        if buttons[LogitechButtons.RIGHT_STICK.value]:
            self.action = "Undock"
            return
        
        if buttons[LogitechButtons.LEFT_STICK.value]:
            self.action = "Dock"
            return

        # If the left trigger is pressed, command the robot to sit
        if buttons[LogitechButtons.A.value]:
            self.action = "ToggleStand"
            return
        
        # If the Y button is pressed, command the robot to power on
        if buttons[LogitechButtons.Y.value]:
            self.action = "PowerOn"
            return
        
        # Up on the DPad to unstow the arm
        if axes[LogitechAxes.DPAD_VERTICAL.value] == 1.0:
            self.action = "ArmUnstow"
            return

        # Down on the DPad to stow the arm
        if axes[LogitechAxes.DPAD_VERTICAL.value] == -1.0:
            self.action = "ArmStow"
            return
        
        # X button to toggle the gripper
        if buttons[LogitechButtons.X.value]:
            self.action = "ToggleGripper"
            return
        
        # LT button activates body pose control
        if buttons[LogitechButtons.LT.value]:
            self.action = "BodyPoseControl"
            self._body_offset = interpolate(axes[LogitechAxes.LEFT_VERTICAL.value]   , [-1.0, 1.0], [-0.15, 0.15])
            self._body_pitch  = interpolate(axes[LogitechAxes.RIGHT_VERTICAL.value]  , [-1.0, 1.0], [-20.0, 20.0])
            self._body_yaw    = interpolate(axes[LogitechAxes.RIGHT_HORIZONTAL.value], [-1.0, 1.0], [-30.0, 30.0])
            return

        self.action = None

    def estopJoyCallback(self, data: Joy):
        buttons = data.buttons
        axes    = data.axes

        # Even for EStop we need to do this check, since the EStop button changes depending on the mode
        if len(axes) != 6:
            return
        
        # If all four letter buttons are pressed, as well as both bumpers, trigger the hard estop
        if buttons[LogitechButtons.A.value] and buttons[LogitechButtons.B.value] and buttons[LogitechButtons.X.value] and buttons[LogitechButtons.Y.value] and buttons[LogitechButtons.RB.value] and buttons[LogitechButtons.LB.value]:
            self.TriggerEStop(hard=True)
            return

        # If the red button is pressed, trigger the soft estop
        if buttons[LogitechButtons.B.value]:
            self.TriggerEStop(hard=False)
            return

        
    def dockRobot(self):
        if self.dock_client is None or not self.verifyServer(self.dock_client):
            self.get_logger.warn("Cannot dock robot, no available server")
            return

        self.get_logger().info("Docking robot")
        resp_future: Future = self.dock_client.call_async(Dock.Request(dock_id=520))
        start_time = self.get_clock().now()
        max_duration = rclpy.duration.Duration(seconds=25)
        while True:
            if resp_future.done():
                resp = resp_future.result()
                break
            elif (self.get_clock().now() - start_time) > max_duration:
                resp = Dock.Response()
                resp.success = False
                resp.message = "Dock server response timed out after 25 seconds"
                break

            time.sleep(0.1)

        self.get_logger().info(f"Success: {resp.success}. Message: {resp.message}")

    def triggerClient(self, client: Client | None):
        if client is None or not self.verifyServer(client):
            return
        
        resp_future = client.call_async(Trigger.Request())
        start_time = self.get_clock().now()
        max_duration = rclpy.duration.Duration(seconds=10)
        while True:
            if resp_future.done():
                resp = resp_future.result()
                break
            elif (self.get_clock().now() - start_time) > max_duration:
                resp = Trigger.Response()
                resp.success = False
                resp.message = f"{client.srv_name} server response timed out after 10 seconds"
                break

            time.sleep(0.1)

        self.get_logger().info(f"Success: {resp.success}. Message: {resp.message}")

    def toggleStand(self):
        if not self.verifyServer(self.stand_client) or not self.verifyServer(self.sit_client):
            self.get_logger.warn("Cannot stand/sit robot, no available server")
            return

        if self._sitting:
            self.get_logger().info("Standing robot")
            self.triggerClient(self.stand_client)
            time.sleep(2.0)

        else:
            self.get_logger().info("Sitting robot")
            self.triggerClient(self.sit_client)
            time.sleep(2.0)

    def toggleGripper(self):
        if not self.verifyServer(self.gripper_open_client) or not self.verifyServer(self.gripper_close_client):
            self.get_logger.warn("Cannot open/close gripper, no available server")
            return

        if self._gripper_closed:
            self.get_logger().info("Opening Gripper")
            self.triggerClient(self.gripper_open_client)

        else:
            self.get_logger().info("Closing Gripper")
            self.triggerClient(self.gripper_close_client)

    def TriggerEStop(self, hard=False):  
        client = self.estop_client_hard if hard else self.estop_client_gentle
        self.get_logger().warn(f"Triggering {'hard' if hard else 'soft'} e-stop")
        
        if client is not None:
            if not self.verifyServer(client):
                return
            self._estop_future = client.call_async(Trigger.Request())

def main():
    rclpy.init()
    node = SpotJoyUtils()

    executor = MultiThreadedExecutor(2) 
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()
