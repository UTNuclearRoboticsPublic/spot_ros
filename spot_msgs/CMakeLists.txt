cmake_minimum_required(VERSION 3.5)
project(spot_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmImpedanceCommand.action"
  "action/ArmSurfaceContact.action"
  "action/ArmCartesianCommand.action"
  "action/ClawGripperCommand.action"
  "action/NavigateTo.action"
  "action/WalkTo.action"
  "action/ImageToGrasp.action"

  "srv/ArmForceTrajectory.srv"
  "srv/ArmJointMovement.srv"
  "srv/ClearBehaviorFault.srv"
  "srv/ConstrainedArmJointMovement.srv"
  "srv/Dock.srv"
  "srv/ForceTrajectory.srv"
  "srv/GripperAngleMove.srv"
  "srv/HandPose.srv"
  "srv/InverseKinematics.srv"
  "srv/ListGraph.srv"
  "srv/NonContactTrajectory.srv"
  "srv/SetLocomotion.srv"
  "srv/SetSimulatedPose.srv"
  "srv/SetVelocity.srv"
  "srv/GestureSequence.srv"
  "srv/GetImages.srv"
  "srv/TogglePayload.srv"
  "srv/RegisterPayload.srv"

  "msg/ArmJointPosition.msg"
  "msg/BatteryState.msg"
  "msg/BatteryStateArray.msg"
  "msg/BehaviorFault.msg"
  "msg/BehaviorFaultState.msg"
  "msg/ClawGripperRequest.msg"
  "msg/DockState.msg"
  "msg/EStopState.msg"
  "msg/EStopStateArray.msg"
  "msg/Feedback.msg"
  "msg/FootState.msg"
  "msg/FootStateArray.msg"
  "msg/Lease.msg"
  "msg/LeaseArray.msg"
  "msg/LeaseOwner.msg"
  "msg/LeaseResource.msg"
  "msg/ManipulatorCarryState.msg"
  "msg/ManipulatorState.msg"
  "msg/ManipulatorStowState.msg"
  "msg/Metrics.msg"
  "msg/MobilityParams.msg"
  "msg/PayloadMassVolumeProperties.msg"
  "msg/PowerState.msg"
  "msg/ScalarTrajectory.msg"
  "msg/ScalarTrajectoryPoint.msg"
  "msg/SE3Trajectory.msg"
  "msg/SE3TrajectoryPoint.msg"
  "msg/SoftwareVersion.msg"
  "msg/SystemFault.msg"
  "msg/SystemFaultState.msg"
  "msg/WiFiState.msg"
  "msg/Gesture.msg"

  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs std_srvs sensor_msgs tf2_msgs trajectory_msgs shape_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
