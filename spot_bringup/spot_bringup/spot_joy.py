from enum import Enum

import time
import rclpy
import numpy as np
from asyncio import Future

import rclpy.duration
from rclpy.node import Node, Parameter
from rclpy.client import Client
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.parameter import ParameterType
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import Joy
from spot_msgs.srv import Dock
from spot_msgs.msg import Feedback, ManipulatorStowState
from std_msgs.msg import Float32
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

class Dualsense5Buttons(Enum):
    CROSS = 1
    SQUARE = 0
    CIRCLE = 2
    TRIANGLE = 3
    L1 = 4
    R1 = 5
    L2 = 6
    R2 = 7
    SHARE = 8
    OPTIONS = 9
    LEFT_STICK = 10
    RIGHT_STICK = 11
    PLAYSTATION = 12
    TOUCHPAD = 13

class Dualsense5Axes(Enum):
    LEFT_HORIZONTAL = 0
    LEFT_VERTICAL = 1
    RIGHT_HORIZONTAL = 2
    RIGHT_VERTICAL = 5
    LEFT_TRIGGER = 3
    RIGHT_TRIGGER = 4
    DPAD_HORIZONTAL = 6
    DPAD_VERTICAL = 7

class LogitechAxes(Enum):
    LEFT_HORIZONTAL  = 0
    LEFT_VERTICAL    = 1
    RIGHT_HORIZONTAL = 2
    RIGHT_VERTICAL   = 3
    DPAD_HORIZONTAL  = 4
    DPAD_VERTICAL    = 5

LogitechActions = {
    "ButtonType": LogitechButtons,
    "AxisType": LogitechAxes,
    "HardEstop": [
        LogitechButtons.A, 
        LogitechButtons.B, 
        LogitechButtons.X, 
        LogitechButtons.Y, 
        LogitechButtons.RB, 
        LogitechButtons.LB
    ],
    "SoftEstop": [LogitechButtons.B],
    "FreezeEstop": [LogitechButtons.A],
    "Claim": [LogitechButtons.START],
    "Release": [LogitechButtons.BACK],
    "PowerOn": [LogitechButtons.Y],
    "Undock": [LogitechButtons.RIGHT_STICK],
    "Dock": [LogitechButtons.LEFT_STICK],
    "BodyPoseControl": [
        LogitechButtons.LT,
        LogitechAxes.LEFT_VERTICAL, -1.0, 1.0,
        LogitechAxes.RIGHT_VERTICAL, -1.0, 1.0,
        LogitechAxes.RIGHT_HORIZONTAL, 1.0, -1.0],
    "ArmUnstow": [LogitechAxes.DPAD_HORIZONTAL, -1.0],
    "ArmStow": [LogitechAxes.DPAD_HORIZONTAL, 1.0],
    "Sit": [LogitechAxes.DPAD_VERTICAL, 1.0],
    "Stand": [LogitechAxes.DPAD_VERTICAL, -1.0],
    "ToggleGripper": [LogitechButtons.X],
}

Dualsense5Actions = {
    "ButtonType": Dualsense5Buttons,
    "AxisType": Dualsense5Axes,
    "HardEstop": [
        Dualsense5Buttons.CROSS, 
        Dualsense5Buttons.CIRCLE, 
        Dualsense5Buttons.SQUARE, 
        Dualsense5Buttons.TRIANGLE, 
        Dualsense5Buttons.R1, 
        Dualsense5Buttons.L1
    ],
    "SoftEstop": [Dualsense5Buttons.CIRCLE],
    "FreezeEstop": [Dualsense5Buttons.CROSS],
    "Claim": [Dualsense5Buttons.OPTIONS],
    "Release": [Dualsense5Buttons.SHARE],
    "PowerOn": [Dualsense5Buttons.TRIANGLE],
    "Undock": [Dualsense5Buttons.RIGHT_STICK],
    "Dock": [Dualsense5Buttons.LEFT_STICK],
    "BodyPoseControl": [
        Dualsense5Buttons.L2,
        Dualsense5Axes.LEFT_VERTICAL, -1.0, 1.0,
        Dualsense5Axes.RIGHT_VERTICAL, -1.0, 1.0,
        Dualsense5Axes.RIGHT_HORIZONTAL, -1.0, 1.0],
    "ArmUnstow": [Dualsense5Axes.DPAD_HORIZONTAL, -1.0],
    "ArmStow": [Dualsense5Axes.DPAD_HORIZONTAL, 1.0],
    "Sit": [Dualsense5Axes.DPAD_VERTICAL, -1.0],
    "Stand": [Dualsense5Axes.DPAD_VERTICAL, 1.0],
    "ToggleGripper": [Dualsense5Buttons.SQUARE],
}

ACTIONS = {'Logitech': LogitechActions, 'Dualsense5': Dualsense5Actions}

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

        # Docking configuration
        self.dock_id: int = self.declare_parameter(name='dock_id',
            value=520,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Configured dock for the robot',
                read_only=False
            )
        ).value
        self.get_logger().info(f'Registering dock id {self.dock_id}')

        # Controller configuration
        self.controller_config: str = self.declare_parameter(name="controller",
            value=Parameter.Type.STRING,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Name of the controller configuration to load",
                read_only=True,  
            )
        ).value

        if self.controller_config not in ACTIONS.keys():
            self.get_logger().error(f"Invalid controller configuration. Valid values are {ACTIONS.keys()}")
            raise RuntimeError()

        self.actions = ACTIONS[self.controller_config]

        self.add_on_set_parameters_callback(self.parameterReconfigureCallback)

        # Subscribe to the feedback topic to monitor dock state
        self._feedback_sub = self.create_subscription(Feedback, '/spot_driver/status/feedback', self.updateState, 10)
        self._arm_stow_sub = self.create_subscription(ManipulatorStowState, '/spot_manipulation_driver/manipulator_state/stow_state', self.updateArmStowState, 10)
        self._gripper_sub  = self.create_subscription(Float32, '/spot_manipulation_driver/manipulator_state/gripper_open_percentage', self.updateArmGripperState, 10)

        exclusive_group = MutuallyExclusiveCallbackGroup()
        self.lease_client = self.create_client(Trigger, "/spot_driver/claim", callback_group=exclusive_group)
        self.release_client = self.create_client(Trigger, "/spot_driver/release", callback_group=exclusive_group)
        self.estop_client_gentle = self.create_client(Trigger, "/spot_driver/estop/gentle")
        self.estop_client_hard = self.create_client(Trigger, "/spot_driver/estop/hard")
        self.freeze_client = self.create_client(Trigger, "/spot_driver/estop/freeze")
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

    def parameterReconfigureCallback(self, parameters: list[Parameter]):
        for param in parameters:
            if param.name == 'dock_id':
                if param.value < 0:
                    self.get_logger().warn(f'Dock id parameter must be a positive integer, but you gave {param.value}')
                    return SetParametersResult(successful=False, reason='Value was outside of the valid range (positive integers)')
                self.dock_id = param.value
                self.get_logger().info(f'Changing configured dock id to {self.dock_id}')
        
        return SetParametersResult(successful=True)

    def updateState(self, msg: Feedback):
        self._docked = msg.docked
        self._sitting = msg.sitting

    def updateArmStowState(self, msg: ManipulatorStowState):
        self._arm_stowed = (msg.state == ManipulatorStowState.STOWSTATE_STOWED) 

    def updateArmGripperState(self, msg: Float32):
        self._gripper_closed = (msg.data < 70.0)
        
    def verifyServer(self, client) -> bool:
        if not client.wait_for_service(5):
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
            elif self.action == "Stand":
                self.get_logger().info("Standing robot")
                client = self.stand_client
            elif self.action == "Sit":
                self.get_logger().info("Sitting robot")
                client = self.sit_client
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

            future = self.triggerClient(client)
            if future is not None:
                self.waitForTriggerFuture(client, future)

        self.action = None

    def checkAction(self, buttons: list[int], axes: list[float], action_name: str):
        action: dict[str: list[Enum]] = self.actions[action_name]

        button_type = self.actions["ButtonType"]
        axis_type = self.actions["AxisType"]

        action_buttons = [buttons[controller_input.value] for controller_input in action if type(controller_input) == button_type]
        action_axes = [axes[controller_input.value] for controller_input in action if type(controller_input) == axis_type]
        action_axis_values = [controller_input for controller_input in action if type(controller_input) == float]
        
        return all(action_buttons) and all(np.array(action_axes) == np.array(action_axis_values))

    def joyCallback(self, data: Joy):
        buttons = data.buttons
        axes    = data.axes

        if len(buttons) != len(self.actions["ButtonType"]):
            self.get_logger().error(f"Wrong controller configuration. Current setting is [{self.controller_config}] but that has {len(self.actions['ButtonType'])} buttons but your controller has {len(buttons)}", throttle_duration_sec=3.0)
            return
        elif len(axes) != (len(self.actions["AxisType"])):
            self.get_logger().error(f"Wrong controller configuration. Current setting is [{self.controller_config}] but that has {len(self.actions['AxisType'])} axes but your controller has {len(axes)}", throttle_duration_sec=3.0)
            return
        # When using Logitech, we need the controller in "D" mode, not "X" mode
        elif self.actions["ButtonType"] == LogitechButtons and len(axes) != 6:
            self.get_logger().warn("Logitech controller in wrong working mode. Please flip the switch on the back", throttle_duration_sec=3.0)
            return

        # Handle actions with a simple trigger format
        simple_actions = ["Claim", "Release", "Undock", "Dock", "Sit", "Stand", "PowerOn", "ArmUnstow", "ArmStow", "ToggleGripper"]
        for action in simple_actions:
            if self.checkAction(buttons, axes, action):
                self.action = action
                return
        
        # Handle the body pose control case
        enable_body_pose_control = buttons[self.actions["BodyPoseControl"][0].value]
        if enable_body_pose_control:
            self.action = "BodyPoseControl"
            body_pose_axes = self.actions["BodyPoseControl"][1::3]
            body_pose_mins = self.actions["BodyPoseControl"][2::3]
            body_pose_maxs = self.actions["BodyPoseControl"][3::3]
            self._body_offset = interpolate(axes[body_pose_axes[0].value], [body_pose_mins[0], body_pose_maxs[0]], [-0.15, 0.15])
            self._body_pitch  = interpolate(axes[body_pose_axes[1].value], [body_pose_mins[1], body_pose_maxs[1]], [-20.0, 20.0])
            self._body_yaw    = interpolate(axes[body_pose_axes[2].value], [body_pose_mins[2], body_pose_maxs[2]], [-30.0, 30.0])
            return
        self.action = None

    def estopJoyCallback(self, data: Joy):
        buttons = data.buttons
        axes    = data.axes

        # Even for EStop we need to do this check, since the EStop button changes depending on the mode
        if self.actions["ButtonType"] == LogitechButtons and len(axes) != 6:
            return
        
        # If all four letter buttons are pressed, as well as both bumpers, trigger the hard estop
        if self.checkAction(buttons, axes, "HardEstop"):
            self.TriggerEStop(hard=True)
            return

        # If the red button is pressed, trigger the soft estop
        if self.checkAction(buttons, axes, "SoftEstop"):
            self.TriggerEStop(hard=False)
            return
        
        # If the green button is pressed, trigger the freeze estop
        if self.checkAction(buttons, axes, "FreezeEstop"):
            self.get_logger().info("Freezing robot")
            self._estop_future = self.triggerClient(self.freeze_client)
            return
        
    def dockRobot(self):
        if self.dock_client is None or not self.verifyServer(self.dock_client):
            self.get_logger().warn("Cannot dock robot, no available server")
            return

        self.get_logger().info("Docking robot")
        resp_future: Future = self.dock_client.call_async(Dock.Request(dock_id=self.dock_id))
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

    def triggerClient(self, client: Client | None) -> Future:
        if client is None or not self.verifyServer(client):
            return
        
        resp_future = client.call_async(Trigger.Request())
        return resp_future

    def waitForTriggerFuture(self, client: Client, future: Future, timeout_sec: int = 10) -> None:
        start_time = self.get_clock().now()
        max_duration = rclpy.duration.Duration(seconds=timeout_sec)
        while True:
            if future.done():
                resp = future.result()
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
            self.get_logger().warn("Cannot stand/sit robot, no available server")
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
            self.get_logger().warn("Cannot open/close gripper, no available server")
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
