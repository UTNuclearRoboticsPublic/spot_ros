import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from spot_msgs.srv import Dock
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from rclpy.action import ActionClient
from geometry_msgs.msg import Vector3
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from spot_msgs.msg import ManipulatorState, ManipulatorStowState, ManipulatorCarryState
from sensor_msgs.msg import JointState

class SimulatedServers(Node):
    def __init__(self):
        super().__init__('spot_simulated_servers')
        self.moveit_client = ActionClient(self, MoveGroup, '/spot_moveit/move_action')
        if not self.moveit_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Unable to contact moveit server')

        self.dock_group = MutuallyExclusiveCallbackGroup()
        self.dock_server = self.create_service(Dock, '/spot_driver/dock', self.handle_dock, callback_group=self.dock_group)
        self.undock_server = self.create_service(Trigger, '/spot_driver/undock', self.handle_undock, callback_group=self.dock_group)

        self.arm_group = MutuallyExclusiveCallbackGroup()
        self.stow_server = self.create_service(Trigger, '/spot_manipulation_driver/stow_arm', self.handle_stow, callback_group=self.arm_group)
        self.unstow_server = self.create_service(Trigger, '/spot_manipulation_driver/mini_unstow_arm', self.handle_unstow, callback_group=self.arm_group)

        self.gripper_open_server = self.create_service(Trigger, '/spot_manipulation_driver/open_gripper', self.handle_open_gripper, callback_group=self.arm_group)
        self.gripper_close_server = self.create_service(Trigger, '/spot_manipulation_driver/close_gripper', self.handle_close_gripper, callback_group=self.arm_group)

        self.stow_state_pub = self.create_publisher(ManipulatorStowState, '/spot_manipulation_driver/manipulator_state/stow_state', 10)
        self.carry_state_pub = self.create_publisher(ManipulatorCarryState, '/spot_manipulation_driver/manipulator_state/carry_state', 10)
        self.gripper_state_pub = self.create_publisher(Float32, '/spot_manipulation_driver/manipulator_state/gripper_open_percentage', 10)

        self.joint_state_pub = self.create_publisher(JointState, '/spot_driver/joint_states', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/spot_driver/joint_states', self.publishManipulatorState, 10)

    def handle_dock(self, req: Dock.Request, resp: Dock.Response) -> Dock.Response:
        resp.success = True
        resp.message = "Simulated dock complete"
        return resp

    def handle_undock(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        resp.success = True
        resp.message = "Simulated undock complete"
        return resp

    def handle_stow(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('Unstowing arm')
        motion_req = MoveGroup.Goal()
        goal = Constraints()
        goal.name = 'stowed'
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_shoulder_yaw', weight=1.0, position=0.0))
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_shoulder_pitch', weight=1.0, position=-3.14159265))
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_elbow_pitch', weight=1.0, position=3.12))
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_elbow_roll', weight=1.0, position=0.00))
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_wrist_pitch', weight=1.0, position=0.00))
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_wrist_roll', weight=1.0, position=0.00))
        motion_req.request.goal_constraints.append(goal)
        motion_req.request.group_name = 'arm'
        motion_req.request.max_velocity_scaling_factor = 1.0
        motion_req.request.max_acceleration_scaling_factor = 1.0
        motion_req.request.workspace_parameters.header.frame_id = 'arm0_base_link'
        motion_req.request.workspace_parameters.min_corner = Vector3(x=-10.0, y=-10.0, z=-10.0)
        motion_req.request.workspace_parameters.max_corner = Vector3(x= 10.0, y= 10.0, z= 10.0)
        motion_req.planning_options.planning_scene_diff.is_diff = True
        motion_req.planning_options.planning_scene_diff.robot_state.is_diff = True
        motion_req.request.start_state.is_diff = True

        self.moveit_client.send_goal_async(motion_req)
        time.sleep(2.0)

        resp.success = True
        resp.message = "Simulated stow requested"
        return resp

    def handle_unstow(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self.get_logger().info('Stowing arm')
        motion_req = MoveGroup.Goal()
        goal = Constraints()
        goal.name = 'unstowed'
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_shoulder_yaw', weight=1.0, position=0.0))
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_shoulder_pitch', weight=1.0, position=-2.356))
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_elbow_pitch', weight=1.0, position=2.566))
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_elbow_roll', weight=1.0, position=0.07))
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_wrist_pitch', weight=1.0, position=-0.21))
        goal.joint_constraints.append(JointConstraint(joint_name='arm0_wrist_roll', weight=1.0, position=-0.07))
        motion_req.request.goal_constraints.append(goal)
        motion_req.request.group_name = 'arm'
        motion_req.request.max_velocity_scaling_factor = 1.0
        motion_req.request.max_acceleration_scaling_factor = 1.0
        motion_req.request.workspace_parameters.header.frame_id = 'arm0_base_link'
        motion_req.request.workspace_parameters.min_corner = Vector3(x=-10.0, y=-10.0, z=-10.0)
        motion_req.request.workspace_parameters.max_corner = Vector3(x= 10.0, y= 10.0, z= 10.0)
        motion_req.planning_options.planning_scene_diff.is_diff = True
        motion_req.planning_options.planning_scene_diff.robot_state.is_diff = True
        motion_req.request.start_state.is_diff = True

        self.moveit_client.send_goal_async(motion_req)
        time.sleep(1.0)

        resp.success = True
        resp.message = "Simulated unstow requested"
        return resp
    
    def handle_open_gripper(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        resp.success = True
        resp.message = 'Pretend the fingers are now open'
        return resp
    
    def handle_close_gripper(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        resp.success = True
        resp.message = 'Pretend the fingers are now closed'
        return resp
    
    def publishManipulatorState(self, joint_state: JointState):
        manipulator_state = ManipulatorState()
        manipulator_state.carry_state = ManipulatorState.CARRY_STATE_CARRIABLE_AND_STOWABLE
        manipulator_state.is_gripper_holding_item = False
        manipulator_state.gripper_open_percentage = 0.0

        base_idx = joint_state.name.index('arm0_shoulder_yaw')
        pitch_idx_1 = joint_state.name.index('arm0_shoulder_pitch')
        pitch_idx_2 = joint_state.name.index('arm0_elbow_pitch')

        base_angle = joint_state.position[base_idx]
        pitch_angle_1 = joint_state.position[pitch_idx_1]
        pitch_angle_2 = joint_state.position[pitch_idx_2]

        eps = 0.01
        if (abs(base_angle) < eps) and (abs(pitch_angle_1 + math.pi) < eps) and (abs(pitch_angle_2 - 3.12) < eps):
            manipulator_state.stow_state = ManipulatorState.STOWSTATE_STOWED
        else:
            manipulator_state.stow_state = ManipulatorState.STOWSTATE_DEPLOYED

        self.stow_state_pub.publish(ManipulatorStowState(state=manipulator_state.stow_state))
        self.carry_state_pub.publish(ManipulatorCarryState(state=manipulator_state.carry_state))
        self.gripper_state_pub.publish(Float32(data=manipulator_state.gripper_open_percentage))
    
def main():
    rclpy.init()
    server_node = SimulatedServers()
    exec = MultiThreadedExecutor()
    exec.add_node(server_node)
    exec.spin()

if __name__ == '__main__':
    main()