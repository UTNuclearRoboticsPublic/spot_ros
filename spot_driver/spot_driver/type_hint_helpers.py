from bosdyn.api import image_pb2, robot_state_pb2, service_fault_pb2, point_cloud_pb2, geometry_pb2, arm_command_pb2
from bosdyn.api.docking import docking_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2

# Until python type hints gets proper support for protobuf types
class Vec3Proto(type[geometry_pb2.Vec3]): pass
class SE2PoseProto(type[geometry_pb2.SE2Pose]): pass
class QuaternionProto(type[geometry_pb2.Quaternion]): pass
class SE3VelocityProto(type[geometry_pb2.SE3Velocity]): pass
class ImageResponseProto(type[image_pb2.ImageResponse]): pass
class PointCloudResponseProto(type[point_cloud_pb2.PointCloudResponse]): pass
class KinematicStateProto(type[robot_state_pb2.KinematicState]): pass
class EStopStateProto(type[robot_state_pb2.EStopState]): pass
class FootStateProto(type[robot_state_pb2.FootState]): pass
class DockStateProto(type[docking_pb2.DockState]): pass
class CommsStateProto(type[robot_state_pb2.CommsState]): pass
class BatteryStateProto(type[robot_state_pb2.BatteryState]): pass
class ServiceFaultProto(type[service_fault_pb2.ServiceFault]): pass
class PowerStateProto(type[robot_state_pb2.PowerState]): pass
class SystemFaultStateProto(type[robot_state_pb2.SystemFaultState]): pass
class BehaviorFaultStateProto(type[robot_state_pb2.BehaviorFaultState]): pass
class BodyExternalParamsProto(type[spot_command_pb2.BodyExternalForceParams]): pass
class MobilityParamsProto(type[spot_command_pb2.MobilityParams]): pass
class ManipulatorStateProto(type[robot_state_pb2.ManipulatorState]): pass
class ArmVelocityCommandProto(type[arm_command_pb2.ArmVelocityCommand]): pass
class ArmCartesianCommandProto(type[arm_command_pb2.ArmCartesianCommand]): pass
class WrenchProto(type[geometry_pb2.Wrench]): pass