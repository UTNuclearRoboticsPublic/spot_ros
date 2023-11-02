from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Joy
from spot_msgs.srv import Dock
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


        exclusive_group = MutuallyExclusiveCallbackGroup()
        self.lease_client = self.create_client(Trigger, "/spot_driver/claim", callback_group=exclusive_group)
        self.dock_client = self.create_client(Dock, "/spot_driver/dock", callback_group=exclusive_group)
        self.undock_client = self.create_client(Trigger, "/spot_driver/undock", callback_group=exclusive_group)
        self.power_on_client = self.create_client(Trigger, "/spot_driver/power_on", callback_group=exclusive_group)
        self.stand_client = self.create_client(Trigger, "/spot_driver/stand", callback_group=exclusive_group)
        self.sit_client = self.create_client(Trigger, "/spot_driver/sit", callback_group=exclusive_group)
        self.unstow_client = self.create_client(Trigger, "/spot_arm_driver/unstow", callback_group=exclusive_group)
        self.stow_client = self.create_client(Trigger, "/spot_arm_driver/stow", callback_group=exclusive_group)

        exclusive_group_2 = MutuallyExclusiveCallbackGroup()
        self.loop = self.create_timer(0.2, self.timerCallback, callback_group=exclusive_group_2)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joyCallback, 10, callback_group=exclusive_group_2)
        self.action = None

        self.get_logger().info("Spot joy node setup complete")

    def verifyClient(self, client) -> bool:
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

            if client is not None:
                if not self.verifyClient(client):
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

        # If the right trigger is pressed, command the robot to stand
        if buttons[LogitechButtons.RT.value]:
            self.action = "Stand"
            return

        # If the left trigger is pressed, command the robot to sit
        if buttons[LogitechButtons.LT.value]:
            self.action = "Sit"
            return
        
        # If the Y button is pressed, command the robot to power on
        if buttons[LogitechButtons.Y.value]:
            self.action = "PowerOn"
            return

        self.action = None
        

    # If docked -> undock. If undocked -> dock
    def toggleDock(self):
        # First try to undock. If this fails, try to dock
        req = Trigger.Request()
        self.get_logger().info("Trying undock")
        if not self.verifyClient(self.undock_client):
            return
        resp = self.undock_client.call(req)

        if not resp.success:
            self.get_logger().info(f"Cannot undock: {resp.message}. Trying dock")
            req = Dock.Request()
            req.dock_id = 520
            if not self.verifyClient(self.dock_client):
                return
            resp = self.dock_client.call(req)
            self.get_logger().info(f"Dock result: {resp.message}")
        else:
            self.get_logger().info(f"Successfully undocked: {resp.message}")

        

def main():
    rclpy.init()
    node = SpotJoyUtils()

    executor = MultiThreadedExecutor(2) 
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()
