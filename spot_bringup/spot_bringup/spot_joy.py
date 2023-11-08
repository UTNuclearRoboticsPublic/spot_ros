from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Joy
from spot_msgs.srv import Dock
from spot_msgs.msg import Feedback, ManipulatorState
from std_srvs.srv import Trigger

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
        self._arm_feedback_sub = self.create_subscription(ManipulatorState, '/follow_joint_trajectory_node/manipulator_state', self.updateArmState, 10)

        exclusive_group = MutuallyExclusiveCallbackGroup()
        self.lease_client = self.create_client(Trigger, "/spot_driver/claim", callback_group=exclusive_group)
        self.dock_client = self.create_client(Dock, "/spot_driver/dock", callback_group=exclusive_group)
        self.undock_client = self.create_client(Trigger, "/spot_driver/undock", callback_group=exclusive_group)
        self.power_on_client = self.create_client(Trigger, "/spot_driver/power_on", callback_group=exclusive_group)
        self.stand_client = self.create_client(Trigger, "/spot_driver/stand", callback_group=exclusive_group)
        self.sit_client = self.create_client(Trigger, "/spot_driver/sit", callback_group=exclusive_group)
        self.unstow_client = self.create_client(Trigger, "/follow_joint_trajectory_node/unstow_arm", callback_group=exclusive_group)
        self.stow_client = self.create_client(Trigger, "/follow_joint_trajectory_node/stow_arm", callback_group=exclusive_group)
        self.gripper_open_client = self.create_client(Trigger, "/follow_joint_trajectory_node/open_gripper", callback_group=exclusive_group)
        self.gripper_close_client = self.create_client(Trigger, "/follow_joint_trajectory_node/close_gripper", callback_group=exclusive_group)

        exclusive_group_2 = MutuallyExclusiveCallbackGroup()
        self.loop = self.create_timer(0.2, self.timerCallback, callback_group=exclusive_group_2)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joyCallback, 10, callback_group=exclusive_group_2)
        self.action = None

        self.get_logger().info("Spot joy node setup complete")

    def updateState(self, msg: Feedback):
        self._docked = msg.docked
        self._sitting = msg.sitting

    def updateArmState(self, msg: ManipulatorState):
        self._arm_stowed = (msg.stow_state == ManipulatorState.STOWSTATE_STOWED) 
        self._gripper_closed = (msg.gripper_open_percentage < 10.0)
        
    def verifyServer(self, client) -> bool:
        if not client.wait_for_service(1):
            self.get_logger().warn(f"Service for action \"{self.action}\" is not available, cancelling request")
            self.action = None
            return False
        return True

    def timerCallback(self):
        if self.action is None:
            return

        if self.action == "ToggleDock":
            self.get_logger().info("Toggling dock")
            self.toggleDock()
        elif self.action == "ToggleStand":
            self.get_logger().info("Toggling stand")
            self.toggleStand()
        elif self.action == "ToggleGripper":
            self.get_logger().info("Toggling gripper")
            self.toggleGripper()

        else:
            if self.action == "Claim":
                self.get_logger().info("Claiming lease")
                client = self.lease_client
            elif self.action == "Sit":
                self.get_logger().info("Sitting")
                client = self.sit_client
            elif self.action == "Stand":
                self.get_logger().info("Standing")
                client = self.stand_client
            elif self.action == "PowerOn":
                self.get_logger().info("Powering on")
                client = self.power_on_client
            elif self.action == "ArmStow":
                self.get_logger().info("Stowing arm")
                client = self.stow_client
            elif self.action == "ArmUnstow":
                self.get_logger().info("Unstowing arm")
                client = self.unstow_client

            if client is not None:
                if not self.verifyServer(client):
                    return
                resp = client.call(Trigger.Request())
                self.get_logger().info(f"Success: {resp.success}. Message: {resp.message}")

        self.action = None


    def joyCallback(self, data: Joy):
        buttons = data.buttons
        axes    = data.axes

        # We need the controller in "D" mode, not "X" mode
        if len(axes) != 6:
            self.get_logger().warn("Logitech controller in wrong working mode. Please flip the switch on the back")

        # If both the start and back buttons are pressed, try to claim a lease
        if buttons[LogitechButtons.START.value] and buttons[LogitechButtons.BACK.value]:
            self.action = "Claim"
            return

        # If both directional sticks are pressed, undock or dock the robot
        if buttons[LogitechButtons.LEFT_STICK.value] and buttons[LogitechButtons.RIGHT_STICK.value]:
            self.action = "ToggleDock"
            return

        # If the left trigger is pressed, command the robot to sit
        if buttons[LogitechButtons.LT.value]:
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

        self.action = None
        
    def toggleDock(self):
        if not self.verifyServer(self.dock_client) or not self.verifyServer(self.undock_client):
            self.get_logger.warn("Cannot dock/undock robot, no available server")

        # First try to undock. If this fails, try to dock
        if self._docked:
            self.get_logger().info("Undocking robot")
            resp = self.undock_client.call(Trigger.Request())
            self.get_logger().info(f"Success: {resp.success}. Message: {resp.message}")

        else:
            self.get_logger().info("Docking robot")
            resp = self.dock_client.call(Dock.Request(dock_id=520))
            self.get_logger().info(f"Success: {resp.success}. Message: {resp.message}")

    def toggleStand(self):
        if not self.verifyServer(self.stand_client) or not self.verifyServer(self.sit_client):
            self.get_logger.warn("Cannot stand/sit robot, no available server")

        if self._sitting:
            self.get_logger().info("Standing robot")
            resp = self.stand_client.call(Trigger.Request())
            self.get_logger().info(f"Success: {resp.success}. Message: {resp.message}")

        else:
            self.get_logger().info("Sitting robot")
            resp = self.sit_client.call(Trigger.Request())
            self.get_logger().info(f"Success: {resp.success}. Message: {resp.message}")

    def toggleGripper(self):
        if not self.verifyServer(self.gripper_open_client) or not self.verifyServer(self.gripper_close_client):
            self.get_logger.warn("Cannot open/close gripper, no available server")

        if self._gripper_closed:
            self.get_logger().info("Opening Gripper")
            resp = self.gripper_open_client.call(Trigger.Request())
            self.get_logger().info(f"Success: {resp.success}. Message: {resp.message}")

        else:
            self.get_logger().info("Closing Gripper")
            resp = self.gripper_close_client.call(Trigger.Request())
            self.get_logger().info(f"Success: {resp.success}. Message: {resp.message}")

def main():
    rclpy.init()
    node = SpotJoyUtils()

    executor = MultiThreadedExecutor(2) 
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()
